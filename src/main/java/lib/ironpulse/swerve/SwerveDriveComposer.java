package lib.ironpulse.swerve;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.SwerveDriveComposerParamsNT;
import lombok.Getter;

/**
 * Force model and software drive velocity loop, ported from 2026-Syscore-robot.
 *
 * <p>The profile states inertial acceleration in the robot frame. The model moves it to the center
 * of mass, forms [Fx, Fy, Mz], and allocates wheel forces with row-scaled damped least squares.
 * Damping keeps parallel and near-parallel wheel directions well behaved.
 *
 * <p>The drive loop composes feedforward current + kP velocity correction + kS smooth-sign friction
 * + kV velocity feedforward. Gains use rotor rotations per second, matching the existing TalonFX
 * slot. Only the P measurement is filtered; model feedforward bypasses it. Commands inside the
 * low-speed deadband produce zero current. The P term is capped at 130 A; the destination's 100 A
 * hardware stator limit still applies to the complete current. Non-finite samples suppress the
 * affected module and raise the fault flag.
 *
 * <p>SwerveLimiter budgets the complete current; TractionGovernor handles per-wheel slip.
 */
public class SwerveDriveComposer {
    /**
     * Damping, as a fraction of the mean eigenvalue of the scaled Gram matrix. Relative rather than
     * absolute so it stays meaningful whatever the chassis geometry is.
     */
    private static final double kDampingFraction = 0.05;

    /** Uncalibrated 10541 feedforward trim; another robot's mass correction does not apply. */
    public static final double kDefaultFeedforwardScale = 1.0;

    /**
     * Bounds on the trim. Wide enough for calibration, tight enough that a typo cannot run away.
     */
    public static final double kMinFeedforwardScale = 0.3;

    public static final double kMaxFeedforwardScale = 1.5;

    /**
     * Default corner for the P term's measurement low pass, Hz. Zero would be the exact bypass; the
     * filter is a bench instrument and the delay levers are the fix — see the class doc.
     */
    public static final double kDefaultPFilterHz = 3.0;

    /** Optional wider filter corner, Hz, retained for live comparison. */
    public static final double kSizedPFilterHz = 10.0;

    /** Zero is not a degenerate corner — it is the documented switch that bypasses the filter. */
    public static final double kMinPFilterHz = 0.0;

    /**
     * Upper bound on the corner. Past this the pole is doing nothing at 200 Hz that raw measurement
     * was not already doing, so a large value is a typo rather than an intent.
     */
    public static final double kMaxPFilterHz = 90.0;

    /**
     * Per wheel bound on the P term alone, amperes — the stopgap floor described on the class,
     * sized at the retired per wheel ceiling.
     */
    public static final double kPClampAmps = 130.0;

    /**
     * Soft-sign width for the static-friction term, in rotor rps. Small, so {@code kS} reaches full
     * effect quickly, but non-zero so it eases through the zero crossing instead of chattering sign
     * on velocity noise. {@code tanh(v/ω₀)} rather than a hard {@code signum}.
     */
    private static final double kFrictionOmegaRps = 2.0;

    private final SwerveConfig config;
    private final int moduleCount;

    /** Module locations re-referenced to the center of mass — the lever arms the wrench sees. */
    private final Translation2d[] leverArms;

    /**
     * Mean lever arm norm, used to row-scale the moment equation. Without it the moment row carries
     * units of length against the two dimensionless force rows, and a single damping term would
     * silently weight force against moment by whatever the chassis happens to measure.
     */
    private final double leverArmScale;

    private final double rotorRpsPerMps;

    /** Commanded drive speed below which a module is held at zero current. See the class doc. */
    private final double lowSpeedDeadbandMps;

    /**
     * Per module low pass state for the P term's measured velocity, m/s. Per module because each
     * wheel rings on its own, and held across the at-rest gate: the gate zeroes the OUTPUT, it does
     * not stop the filter tracking what the wheel is doing.
     */
    private final double[] pFilterState;

    /**
     * Commanded acceleration OF THE CENTER OF MASS as {@code [ax (m/s²), ay (m/s²), α (rad/s²)]}.
     */
    @Getter private final double[] accelerationCmd = new double[3];

    /** Commanded wrench about the center of mass as {@code [Fx (N), Fy (N), Mz (N·m)]}. */
    @Getter private final double[] wrench = new double[3];

    /** Per module wheel force along the commanded azimuth, in newtons, after the trim. */
    @Getter private final double[] forcesNewton;

    /** Whether a non-finite input forced the feedforward to zero this tick. */
    @Getter private boolean fault = false;

    /** Whether the P term's per wheel clamp bound on any module this tick. */
    @Getter private boolean pClamped = false;

    // scratch for the scaled allocation matrix rows, reused every tick
    private final double[] aRow0;
    private final double[] aRow1;
    private final double[] aRow2;

    /** Per-module split of the composed current, in amps, kept for logging: FF, P, friction. */
    @Getter private final double[] ffAmps;

    @Getter private final double[] pAmps;

    @Getter private final double[] frictionAmps;

    private final double[] totalAmps;

    public SwerveDriveComposer(SwerveConfig config) {
        this.config = config;
        this.moduleCount = config.moduleCount();

        this.forcesNewton = new double[moduleCount];
        this.aRow0 = new double[moduleCount];
        this.aRow1 = new double[moduleCount];
        this.aRow2 = new double[moduleCount];

        Translation2d[] moduleLocations = config.moduleLocations();
        this.leverArms = new Translation2d[moduleCount];
        double normSum = 0.0;
        for (int i = 0; i < moduleCount; i++) {
            leverArms[i] = moduleLocations[i].minus(config.comOffset);
            normSum += leverArms[i].getNorm();
        }
        double meanNorm = normSum / moduleCount;
        this.leverArmScale = meanNorm > 0.0 ? meanNorm : 1.0;

        // rotor rps for a given wheel m/s: (mps / wheel-circumference) · gear ratio — the same
        // conversion the firmware velocity request uses, so the gains mean the same thing.
        double wheelCircumference = Math.PI * config.wheelDiameter.in(Meter);
        this.rotorRpsPerMps = config.driveGearRatio / wheelCircumference;
        this.lowSpeedDeadbandMps = config.lowSpeedDeadband.in(MetersPerSecond);

        this.pFilterState = new double[moduleCount];
        this.ffAmps = new double[moduleCount];
        this.pAmps = new double[moduleCount];
        this.frictionAmps = new double[moduleCount];
        this.totalAmps = new double[moduleCount];
    }

    /**
     * The live trim on the feedforward, read fresh each tick and bounded to {@code
     * [kMinFeedforwardScale, kMaxFeedforwardScale]}.
     */
    public static double feedforwardScale() {
        double requested = SwerveDriveComposerParamsNT.scale.getValue();
        if (!Double.isFinite(requested)) return kDefaultFeedforwardScale;
        return edu.wpi.first.math.MathUtil.clamp(
                requested, kMinFeedforwardScale, kMaxFeedforwardScale);
    }

    /**
     * The live corner frequency of the P term's measurement low pass, Hz, bounded to {@code
     * [kMinPFilterHz, kMaxPFilterHz]}. Zero means bypass; see the class doc.
     */
    public static double pFilterHz() {
        double requested = SwerveDriveComposerParamsNT.pFilterHz.getValue();
        if (!Double.isFinite(requested)) return kDefaultPFilterHz;
        return edu.wpi.first.math.MathUtil.clamp(requested, kMinPFilterHz, kMaxPFilterHz);
    }

    /**
     * Single pole coefficient for a corner frequency at this loop period: {@code dt / (dt + tau)}
     * with {@code tau = 1/(2*pi*f)}.
     */
    private static double pFilterAlpha(double cornerHz, double dtS) {
        return dtS / (dtS + 1.0 / (2.0 * Math.PI * cornerHz));
    }

    /**
     * Clears the P term's measurement filter and the logged split. Call on disable — a filter
     * carrying pre-disable velocity into the first re-enabled tick is a kick waiting to happen.
     */
    public void reset() {
        for (int i = 0; i < moduleCount; i++) {
            pFilterState[i] = 0.0;
            ffAmps[i] = 0.0;
            pAmps[i] = 0.0;
            frictionAmps[i] = 0.0;
            totalAmps[i] = 0.0;
        }
        pClamped = false;
    }

    /**
     * One tick of the model: the setpoint's stated acceleration — robot frame, SI, the profile's —
     * to per module wheel force. No filter on this path: the acceleration was correct on arrival,
     * and lagging it would only corrupt it.
     */
    public void compute(SwerveSetpoint setpoint) {
        ChassisSpeeds speeds = setpoint.chassisSpeeds();
        ChassisAccel accel = setpoint.accel();
        double omega = speeds.omegaRadiansPerSecond;

        // Acceleration of the KINEMATICS ORIGIN, robot frame. The profile's is already inertial —
        // its carried vectors are re-expressed through the gyro's yaw increment every tick — so no
        // frame-rotation terms (ω×v) are added here: they would state a force for a motion the
        // wheels only re-head for.
        double ax = accel.axMetersPerSecondSq();
        double ay = accel.ayMetersPerSecondSq();
        double alpha = accel.alphaRadiansPerSecondSq();

        // A NaN anywhere upstream must not be laundered into a motor command — the twist included:
        // only ω enters this model now, but a NaN twist is a broken setpoint either way.
        if (!Double.isFinite(ax)
                || !Double.isFinite(ay)
                || !Double.isFinite(alpha)
                || !Double.isFinite(speeds.vxMetersPerSecond)
                || !Double.isFinite(speeds.vyMetersPerSecond)
                || !Double.isFinite(omega)) {
            zeroOutput(true);
            return;
        }

        // Move it to the center of mass. Iz and the moment row are both referenced there, so using
        // the origin's acceleration for the translational half would mix two reference points and
        // silently mis-state the wrench whenever the chassis is rotating.
        double rcx = config.comOffset.getX();
        double rcy = config.comOffset.getY();
        double omegaSq = omega * omega;
        double axCom = ax + alpha * -rcy - omegaSq * rcx;
        double ayCom = ay + alpha * rcx - omegaSq * rcy;

        accelerationCmd[0] = axCom;
        accelerationCmd[1] = ayCom;
        accelerationCmd[2] = alpha;

        // Inertial wrench only — see the class doc on why friction is not here.
        double fx = config.driveMass.in(Kilograms) * axCom;
        double fy = config.driveMass.in(Kilograms) * ayCom;
        double mz = config.chassisMoiZ.in(KilogramSquareMeters) * alpha;

        wrench[0] = fx;
        wrench[1] = fy;
        wrench[2] = mz;

        // Row-scaled allocation matrix Ã, built column by column together with the symmetric Gram
        // matrix Ã Ãᵀ.
        SwerveModuleState[] states = setpoint.moduleStates();
        boolean finite = Double.isFinite(fx) && Double.isFinite(fy) && Double.isFinite(mz);
        double m00 = 0.0, m01 = 0.0, m02 = 0.0, m11 = 0.0, m12 = 0.0, m22 = 0.0;
        for (int i = 0; i < moduleCount; i++) {
            Rotation2d azimuth = states[i].angle;
            double cos = azimuth.getCos();
            double sin = azimuth.getSin();
            finite &= Double.isFinite(cos) && Double.isFinite(sin);

            aRow0[i] = cos;
            aRow1[i] = sin;
            aRow2[i] = (leverArms[i].getX() * sin - leverArms[i].getY() * cos) / leverArmScale;

            m00 += aRow0[i] * aRow0[i];
            m01 += aRow0[i] * aRow1[i];
            m02 += aRow0[i] * aRow2[i];
            m11 += aRow1[i] * aRow1[i];
            m12 += aRow1[i] * aRow2[i];
            m22 += aRow2[i] * aRow2[i];
        }

        // A NaN anywhere upstream (a stale setpoint, a divide by a zero dt) must not be laundered
        // into a motor command.
        if (!finite) {
            zeroOutput(true);
            return;
        }

        double mzScaled = mz / leverArmScale;

        // Damped least squares. λ² is a fraction of the mean eigenvalue, so damping tracks the
        // problem's scale. The result is positive definite for any azimuths — there is no singular
        // case to branch on.
        double lambdaSq = kDampingFraction * (m00 + m11 + m22) / 3.0;
        double d00 = m00 + lambdaSq;
        double d11 = m11 + lambdaSq;
        double d22 = m22 + lambdaSq;

        // Closed-form inverse of the damped symmetric 3x3 (adjugate / determinant).
        double adj00 = d11 * d22 - m12 * m12;
        double adj01 = m02 * m12 - m01 * d22;
        double adj02 = m01 * m12 - m02 * d11;
        double det = d00 * adj00 + m01 * adj01 + m02 * adj02;
        double adj11 = d00 * d22 - m02 * m02;
        double adj12 = m01 * m02 - d00 * m12;
        double adj22 = d00 * d11 - m01 * m01;

        // y = (Ã Ãᵀ + λ²I)⁻¹ W̃, then f = Ãᵀ y is the damped minimum norm force allocation.
        double y0 = (adj00 * fx + adj01 * fy + adj02 * mzScaled) / det;
        double y1 = (adj01 * fx + adj11 * fy + adj12 * mzScaled) / det;
        double y2 = (adj02 * fx + adj12 * fy + adj22 * mzScaled) / det;

        double scale = feedforwardScale();
        for (int i = 0; i < moduleCount; i++)
            forcesNewton[i] = (aRow0[i] * y0 + aRow1[i] * y1 + aRow2[i] * y2) * scale;
        fault = false;
    }

    private void zeroOutput(boolean isFault) {
        fault = isFault;
        for (int i = 0; i < moduleCount; i++) forcesNewton[i] = 0.0;
    }

    /**
     * Composes the per-module drive current.
     *
     * @param feedforwardAmps per-module feedforward current, amps
     * @param cmdDriveMps per-module commanded drive speed, m/s; a module inside the low speed
     *     deadband is gated to zero current outright
     * @param measDriveMps per-module measured drive speed, m/s; the P term sees this through the
     *     low pass described on the class, the other terms do not see it at all
     * @param kp proportional gain, amps per rotor rps of error
     * @param ks static-friction feedforward, amps
     * @param kv velocity feedforward, amps per rotor rps
     * @return per-module total commanded current, amps — the same buffer each tick
     */
    public double[] compose(
            double[] feedforwardAmps,
            double[] cmdDriveMps,
            double[] measDriveMps,
            double kp,
            double ks,
            double kv) {
        double cornerHz = pFilterHz();
        boolean bypass = cornerHz <= 0.0;
        double alpha = bypass ? 1.0 : pFilterAlpha(cornerHz, config.dtS);

        pClamped = false;
        for (int i = 0; i < moduleCount; i++) {
            if (!Double.isFinite(cmdDriveMps[i])
                    || !Double.isFinite(measDriveMps[i])
                    || !Double.isFinite(feedforwardAmps[i])
                    || !Double.isFinite(kp)
                    || !Double.isFinite(ks)
                    || !Double.isFinite(kv)) {
                ffAmps[i] = 0.0;
                pAmps[i] = 0.0;
                frictionAmps[i] = 0.0;
                totalAmps[i] = 0.0;
                fault = true;
                continue;
            }
            // Ahead of the gate, deliberately: a gated module is still moving and the filter must
            // keep tracking it, or the first tick back over the deadband would difference against a
            // state
            // frozen at whatever the wheel was doing when it went quiet.
            double vMeasFiltered;
            if (bypass) {
                // Exact identity, not a very wide pole — the state follows so a live change of
                // corner starts from the present measurement rather than a stale one.
                pFilterState[i] = measDriveMps[i];
                vMeasFiltered = measDriveMps[i];
            } else {
                // Hold on a non-finite sample rather than latching NaN into the state for the life
                // of the process, the same discipline the model's input filters follow.
                if (Double.isFinite(measDriveMps[i]))
                    pFilterState[i] += alpha * (measDriveMps[i] - pFilterState[i]);
                vMeasFiltered = pFilterState[i];
            }

            // Asked to stand still: command nothing at all, rather than closing a delayed loop on
            // measurement noise. Every term is zeroed, including the logged split, so a bag shows
            // that nothing was commanded instead of a P term that never reached the motor.
            if (Math.abs(cmdDriveMps[i]) < lowSpeedDeadbandMps) {
                ffAmps[i] = 0.0;
                pAmps[i] = 0.0;
                frictionAmps[i] = 0.0;
                totalAmps[i] = 0.0;
                continue;
            }

            double vCmdRps = cmdDriveMps[i] * rotorRpsPerMps;
            double vMeasRps = vMeasFiltered * rotorRpsPerMps;

            double ff = feedforwardAmps[i];
            double p = kp * (vCmdRps - vMeasRps);
            if (Math.abs(p) > kPClampAmps) {
                p = Math.copySign(kPClampAmps, p);
                pClamped = true;
            }
            double friction = ks * Math.tanh(vCmdRps / kFrictionOmegaRps) + kv * vCmdRps;

            ffAmps[i] = ff;
            pAmps[i] = p;
            frictionAmps[i] = friction;
            totalAmps[i] = ff + p + friction;
        }
        return totalAmps;
    }
}
