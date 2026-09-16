package lib.ironpulse.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.SwerveProfileParamsNT;
import lib.ntext.NTParameterWrapper;

/**
 * The chassis reference shaper: one jerk-limited velocity profile that every source — stick,
 * follower, PathPlanner, stops — drives through, and the only thing that states the chassis
 * acceleration.
 *
 * <p>Every tick the profile carries a state {@code (v, a)} for the linear velocity vector and
 * {@code (ω, α)} for yaw, robot frame, and plans from that state to the desired twist at rest —
 * {@code (v_des, 0, 0)}: velocity, acceleration and jerk — with a quartic Hermite over a horizon
 * {@code T}. The quartic is the simplest basis that admits the relaxed-set check below: its initial
 * jerk is free, so a re-plan reverses a ramp at the jerk limit on the tick the target moves and a
 * target that keeps moving (a position law re-evaluated every tick) still builds acceleration. A
 * quintic carrying the jerk as state is viable too, with the jerk relaxed the same way the
 * acceleration is ({@code max(limit, |j₀|)}); it tracks within 1 % of this one and needs longer
 * horizons. The polynomial is evaluated one loop period in, the state is carried, and the evaluated
 * acceleration is handed to {@link SwerveDriveComposer} as the feedforward's intent — through the
 * setpoint generator, which states the commanded twist's own rate (desaturation rate term included;
 * steer-coupled ticks are re-synced). Nothing downstream differentiates a measurement: the
 * acceleration is analytic, continuous, and bounded here; the jerk is bounded and may step where
 * the plan changes.
 *
 * <p>The horizon is chosen so the plan's peak acceleration and jerk fit the limits, and the peaks
 * are the polynomial's own extrema in closed form. A target that has not moved is followed on the
 * plan made for it: the horizon counts down by one period a tick, and a quartic re-fitted from its
 * own intermediate state over its own remaining horizon is the same quartic, so there is nothing to
 * re-check. A target that moved is re-planned from the shorter of the remaining horizon and the
 * closed-form estimate — the rest-to-rest profile {@code v(τ) = Δv(6τ² − 8τ³ + 3τ⁴)}, peak {@code
 * |a| = 1.778·|Δv|/T}, peak {@code |j| = 12·|Δv|/T²}, floored at what the current acceleration
 * needs to unwind to rest within the jerk limit ({@code a(τ) = a₀(1−τ)²(1−4τ)}, {@code |j| ≤
 * 6·|a₀|/T}) — and the horizon is lengthened until the extrema fit, up to {@value #kMaxHorizon} s.
 * The plan is checked against a set that contains its own initial acceleration exactly ({@code
 * max(limit, |a₀|)}): a state already over a limit — the brake set was the tighter one a tick ago,
 * a parameter was lowered live — is unwound, never rejected and never grown, so there is no horizon
 * that can fail forever. Decelerating — the desired speed under the current — plans against the
 * brake set.
 *
 * <p>Velocity caps are applied to the desired twist before the fit, so the profile converges to the
 * cap rather than overshooting it; both caps are bounded by what the modules can actually run
 * ({@code maxDriveVelocity}, and that over the lever arm for yaw), whatever the parameter asks. A
 * hard clamp on the evaluated acceleration — at {@value #kClampFactor}× the limit, or at the
 * magnitude it already had, whichever is larger, so a clamped state can only fall — is the last
 * line. It binds only when no horizon up to {@value #kMaxHorizon} s could fit the plan, it is
 * logged the tick it binds and latched until the next {@link #reset}, and it forces a re-plan.
 *
 * <p>The linear reference is inertial. Its velocity, acceleration and jerk are physical vectors
 * carried across ticks in robot coordinates, and the robot turns under them: before each step
 * {@link #rotate} re-expresses them in the frame the chassis has turned into, by the gyro's yaw
 * increment. A driver holding one field direction while yawing is then a target that does not move,
 * and frame rotation — a motion the wheels only re-head for — costs none of the acceleration
 * budget. Shaping the twist in the rotating frame instead made a constant field velocity a target
 * turning at {@code −ω}, which the bounded plan could only chase: the executed translation swung
 * toward the rotation by tens of degrees at {@code ω·v} above the accel limit (2026-08-25 log). The
 * acceleration stated here is therefore the inertial one in robot coordinates, and the composer
 * adds no frame-rotation terms of its own.
 *
 * <p>Pure: no robot clock, no hardware. {@code dt} comes in, the state is the only memory, and
 * {@link #reset} seeds it from a measured twist so a re-enable never replays a stale plan.
 */
public final class ChassisProfile {
    /** The bounds the profile plans against, SI, robot frame. */
    public record Limits(
            double maxLinearVelocity,
            double maxLinearAccel,
            double maxLinearJerk,
            double maxBrakeAccel,
            double maxBrakeJerk,
            double maxAngularVelocity,
            double maxAngularAccel,
            double maxAngularJerk) {}

    /**
     * The carried reference — velocity and acceleration — and the plan's jerk at it (reported, not
     * carried), robot frame, SI.
     */
    public record State(
            double vx,
            double vy,
            double ax,
            double ay,
            double jx,
            double jy,
            double omega,
            double alpha,
            double zeta) {}

    /** One tick's output: the setpoint twist and the acceleration it was planned with. */
    public record Reference(ChassisSpeeds velocity, ChassisAccel accel) {}

    public static final double kDefaultMaxLinearVelocity = 4.75;

    /**
     * The acceleration and jerk sets are the feel tune (2026-08-25), not a carpet μ: caps shape the
     * reference, traction is {@link TractionGovernor}'s per wheel. 13 m/s² is over the measured 1.4
     * launch μ on purpose.
     */
    public static final double kDefaultMaxLinearAccel = 13.0;

    public static final double kDefaultMaxLinearJerk = 1500.0;
    public static final double kDefaultMaxBrakeAccel = 20.0;
    public static final double kDefaultMaxBrakeJerk = 4000.0;

    /**
     * rad/s. The physical bound: the 5.0 m/s module free speed over the 0.498 m lever arm is 10.03
     * rad/s of pure rotation. The pre-profile 1000 deg/s cap never described the chassis.
     */
    public static final double kDefaultMaxAngularVelocity = 7.0;

    /** rad/s². Inside the ≈41 rad/s² the four wheels can put into yaw at the launch μ. */
    public static final double kDefaultMaxAngularAccel = 40.0;

    public static final double kDefaultMaxAngularJerk = 1000.0;

    /** Peak acceleration of the rest-to-rest quartic, {@code 12τ(1−τ)²} at τ = 1/3, per Δv/T. */
    private static final double kAccelPeakFactor = 16.0 / 9.0;

    /** Peak jerk of the same profile, {@code 12 − 48τ + 36τ²} at τ = 0, per Δv/T². */
    private static final double kJerkPeakFactor = 12.0;

    /**
     * Peak jerk of unwinding an acceleration to rest — {@code a(T) = 0, j(T) = 0, Δv = 0}, {@code
     * a(τ) = a₀(1−τ)²(1−4τ)} — as a multiple of a₀/T: {@code j(0) = −6a₀/T}.
     */
    private static final double kUnwindJerkFactor = 6.0;

    /** The evaluated acceleration may never exceed the limit by more than this factor. */
    public static final double kClampFactor = 1.05;

    /** No plan is longer than this; a horizon that would need more is a fault, not a plan. */
    public static final double kMaxHorizon = 3.0;

    /** A plan's extrema may sit this far over the limit before the horizon is lengthened. */
    private static final double kPeakTolerance = 1.005;

    private static final double kHorizonGrowth = 1.25;
    private static final int kFitIterations = 8;

    /** A desired twist that moved less than this since last tick counts as the same target. */
    private static final double kTargetEpsilon = 1e-6;

    private final double maxDriveVelocity;
    private final double leverArm;
    private final Axis linear = new Axis(2);
    private final Axis angular = new Axis(1);
    private boolean capsBound = false;
    private SwerveLimit limitOverride;

    /** Existing autonomous commands can temporarily replace chassis caps; jerk remains profiled. */
    public void setLimitOverride(SwerveLimit limit) {
        limitOverride = limit;
    }

    /**
     * @param maxDriveVelocity the module free speed the setpoint generator desaturates to, m/s —
     *     the bound on {@code maxLinearVelocity}
     * @param leverArm the farthest module's distance from the kinematics origin, m — pure rotation
     *     at {@code maxDriveVelocity / leverArm} saturates that module
     */
    public ChassisProfile(double maxDriveVelocity, double leverArm) {
        this.maxDriveVelocity = maxDriveVelocity;
        this.leverArm = leverArm;
    }

    /**
     * The live limits, read from {@code Params/Swerve/Profile} and bounded: the velocity caps to
     * what the modules can run, the rest so a typo cannot plan a profile the chassis could never
     * follow. {@link #capsBound()} reports whether a velocity cap had to be cut.
     */
    public Limits limits() {
        double wBound = maxDriveVelocity / leverArm;
        double vReq = SwerveProfileParamsNT.maxLinearVelocity.getValue();
        double wReq = SwerveProfileParamsNT.maxAngularVelocity.getValue();
        SwerveLimit override = limitOverride;
        if (override != null) {
            vReq = override.maxLinearVelocity().in(edu.wpi.first.units.Units.MetersPerSecond);
            wReq = override.maxAngularVelocity().in(edu.wpi.first.units.Units.RadiansPerSecond);
        }
        capsBound = !(vReq <= maxDriveVelocity) || !(wReq <= wBound); // a non-finite read too
        return new Limits(
                bounded(vReq, kDefaultMaxLinearVelocity, 0.1, maxDriveVelocity),
                bounded(
                        override == null
                                ? SwerveProfileParamsNT.maxLinearAccel.getValue()
                                : override.maxSkidAcceleration()
                                        .in(edu.wpi.first.units.Units.MetersPerSecondPerSecond),
                        kDefaultMaxLinearAccel,
                        0.5,
                        50.0),
                bounded(SwerveProfileParamsNT.maxLinearJerk, kDefaultMaxLinearJerk, 5.0, 2000.0),
                bounded(
                        override == null
                                ? SwerveProfileParamsNT.maxBrakeAccel.getValue()
                                : override.maxBrakeAccelerationOrSkid()
                                        .in(edu.wpi.first.units.Units.MetersPerSecondPerSecond),
                        kDefaultMaxBrakeAccel,
                        0.5,
                        50.0),
                bounded(SwerveProfileParamsNT.maxBrakeJerk, kDefaultMaxBrakeJerk, 5.0, 2000.0),
                bounded(wReq, kDefaultMaxAngularVelocity, 0.1, wBound),
                bounded(
                        override == null
                                ? SwerveProfileParamsNT.maxAngularAccel.getValue()
                                : override.maxAngularAcceleration()
                                        .in(edu.wpi.first.units.Units.RadiansPerSecondPerSecond),
                        kDefaultMaxAngularAccel,
                        0.5,
                        200.0),
                bounded(SwerveProfileParamsNT.maxAngularJerk, kDefaultMaxAngularJerk, 5.0, 5000.0));
    }

    /** Whether the last {@link #limits()} read had to cut a velocity cap to the physical bound. */
    public boolean capsBound() {
        return capsBound;
    }

    private static double bounded(
            NTParameterWrapper<Double> param, double fallback, double min, double max) {
        return bounded(param.getValue(), fallback, min, max);
    }

    private static double bounded(double requested, double fallback, double min, double max) {
        return edu.wpi.first.math.MathUtil.clamp(
                Double.isFinite(requested) ? requested : fallback, min, max);
    }

    /**
     * Advances the reference one loop period toward {@code desired}.
     *
     * @param desired the twist the source asks for, robot frame; a non-finite twist is taken as a
     *     request to stop, so a bad command brakes the chassis instead of poisoning the state
     * @param limits the bounds to plan against this tick
     * @param dt the loop period, seconds
     * @return the setpoint twist and its acceleration
     */
    public Reference step(ChassisSpeeds desired, Limits limits, double dt) {
        if (!Double.isFinite(desired.vxMetersPerSecond)
                || !Double.isFinite(desired.vyMetersPerSecond)
                || !Double.isFinite(desired.omegaRadiansPerSecond)) desired = new ChassisSpeeds();

        double vDesNorm = Math.hypot(desired.vxMetersPerSecond, desired.vyMetersPerSecond);
        double vCap = limits.maxLinearVelocity();
        double vScale = vDesNorm > vCap ? vCap / vDesNorm : 1.0;
        double[] vDes = {desired.vxMetersPerSecond * vScale, desired.vyMetersPerSecond * vScale};
        boolean braking = vDesNorm * vScale < Math.hypot(linear.v[0], linear.v[1]);
        linear.step(
                vDes,
                braking ? limits.maxBrakeAccel() : limits.maxLinearAccel(),
                braking ? limits.maxBrakeJerk() : limits.maxLinearJerk(),
                dt);

        double wCap = limits.maxAngularVelocity();
        angular.step(
                new double[] {
                    edu.wpi.first.math.MathUtil.clamp(desired.omegaRadiansPerSecond, -wCap, wCap)
                },
                limits.maxAngularAccel(),
                limits.maxAngularJerk(),
                dt);

        return new Reference(
                new ChassisSpeeds(linear.v[0], linear.v[1], angular.v[0]),
                new ChassisAccel(linear.a[0], linear.a[1], angular.a[0]));
    }

    /**
     * Re-expresses the linear reference in the robot frame after the chassis turned by {@code
     * dThetaRad} (counter-clockwise positive) since the last step. The carried velocity,
     * acceleration and jerk are physical vectors; left alone they would turn with the robot. The
     * fitted plan is rotated with them — a rotated quartic is the quartic fitted to the rotated
     * boundary conditions — so an unmoved field target still replays its plan exactly. Yaw is a
     * scalar and needs nothing.
     */
    public void rotate(double dThetaRad) {
        linear.rotate(dThetaRad);
    }

    /**
     * Seeds the reference at a measured twist with no acceleration or jerk and no plan.
     *
     * @param clearFault whether the latched clamp fault is cleared too — the disable path and the
     *     X-lock do; a plain stop re-seeds and keeps the fault visible until then
     */
    public void reset(ChassisSpeeds measured, boolean clearFault) {
        linear.reset(clearFault, measured.vxMetersPerSecond, measured.vyMetersPerSecond);
        angular.reset(clearFault, measured.omegaRadiansPerSecond);
    }

    /**
     * Pulls the reference's velocity back to the twist the modules were actually commanded, keeping
     * its acceleration, when the steer coupling withheld part of this tick's step (modules still
     * turning); the next tick re-plans from there, so the coupling's release is continuous rather
     * than a jump to the lead the profile built while it waited.
     */
    public void syncVelocity(ChassisSpeeds commanded) {
        linear.sync(commanded.vxMetersPerSecond, commanded.vyMetersPerSecond);
        angular.sync(commanded.omegaRadiansPerSecond);
    }

    public State state() {
        return new State(
                linear.v[0],
                linear.v[1],
                linear.a[0],
                linear.a[1],
                linear.j[0],
                linear.j[1],
                angular.v[0],
                angular.a[0],
                angular.j[0]);
    }

    /** The horizons the last tick planned over, seconds: {@code [linear, angular]}. */
    public double[] horizons() {
        return new double[] {linear.horizon, angular.horizon};
    }

    /** Whether the acceleration clamp bound on the last tick, on either axis group. */
    public boolean clampEngaged() {
        return linear.clamped || angular.clamped;
    }

    /** Whether the clamp has bound at any tick since the last {@link #reset}: the sticky fault. */
    public boolean clampLatched() {
        return linear.clampLatched || angular.clampLatched;
    }

    /**
     * One axis group — the linear vector or the yaw scalar — planned with a shared horizon so the
     * vector's direction is preserved through the profile.
     */
    private static final class Axis {
        final double[] v;
        final double[] a;
        final double[] j;
        double horizon = 0.0;
        boolean clamped = false;
        boolean clampLatched = false;
        private boolean synced = false;
        private double aLimPrev = Double.NaN;
        private double jLimPrev = Double.NaN;

        // quartic coefficients of the current plan, per component: v0 + a0·t + c2·t² + c3·t³ +
        // c4·t⁴
        private final double[] c2;
        private final double[] c3;
        private final double[] c4;
        private final double[] vDes;
        private final double[] vDesPrev;
        private final double[] vPrev;

        Axis(int n) {
            v = new double[n];
            a = new double[n];
            j = new double[n];
            c2 = new double[n];
            c3 = new double[n];
            c4 = new double[n];
            vDes = new double[n];
            vDesPrev = new double[n];
            vPrev = new double[n];
        }

        void reset(boolean clearFault, double... velocity) {
            System.arraycopy(velocity, 0, v, 0, v.length);
            java.util.Arrays.fill(a, 0.0);
            java.util.Arrays.fill(j, 0.0);
            java.util.Arrays.fill(vDesPrev, Double.NaN);
            horizon = 0.0;
            clamped = false;
            if (clearFault) clampLatched = false;
            synced = false;
        }

        void sync(double... velocity) {
            System.arraycopy(velocity, 0, v, 0, v.length);
            synced = true;
        }

        /**
         * Every vector this axis carries, turned by {@code −theta}: the frame turned by {@code
         * +theta}.
         */
        void rotate(double theta) {
            if (v.length != 2 || !Double.isFinite(theta) || theta == 0.0) return;
            double c = Math.cos(theta), s = Math.sin(theta);
            for (double[] vec : new double[][] {v, a, j, c2, c3, c4, vDes, vDesPrev, vPrev}) {
                double x = vec[0], y = vec[1];
                vec[0] = c * x + s * y;
                vec[1] = c * y - s * x;
            }
        }

        void step(double[] target, double aLim, double jLim, double dt) {
            double dv = 0.0, aNow = 0.0, moved = 0.0;
            for (int i = 0; i < v.length; i++) {
                vDes[i] = target[i];
                dv += (vDes[i] - v[i]) * (vDes[i] - v[i]);
                moved += (vDes[i] - vDesPrev[i]) * (vDes[i] - vDesPrev[i]);
                aNow += a[i] * a[i];
                vDesPrev[i] = vDes[i];
            }
            dv = Math.sqrt(dv);
            aNow = Math.sqrt(aNow);

            // An unmoved target on an unclamped plan under the same set: the plan already made,
            // counted down — the same polynomial, so nothing to re-check. Anything else: a re-plan.
            double remaining = horizon - dt;
            boolean replay =
                    remaining >= dt
                            && !clamped
                            && !synced
                            && moved <= kTargetEpsilon * kTargetEpsilon // NaN after reset: false
                            && aLim == aLimPrev
                            && jLim == jLimPrev;
            aLimPrev = aLim;
            jLimPrev = jLim;
            synced = false;
            double t;
            if (replay) {
                t = remaining;
                fit(t);
            } else {
                // The set the plan is checked against contains its own initial acceleration
                // exactly, so an over-limit state can be unwound but never grown.
                double aPlan = Math.max(aLim * kPeakTolerance, aNow);
                double jPlan = jLim * kPeakTolerance;
                double estimate =
                        Math.max(
                                Math.max(dt, kAccelPeakFactor * dv / aPlan),
                                Math.max(
                                        Math.sqrt(kJerkPeakFactor * dv / jPlan),
                                        kUnwindJerkFactor * aNow / jPlan));
                t =
                        Math.min(
                                remaining >= dt ? Math.min(remaining, estimate) : estimate,
                                kMaxHorizon);
                fit(t);
                for (int iter = 0; iter < kFitIterations && t < kMaxHorizon; iter++) {
                    if (peaksFit(t, aPlan, jPlan)) break;
                    t = Math.min(kMaxHorizon, Math.max(estimate, t * kHorizonGrowth));
                    fit(t);
                }
            }
            horizon = t;

            double aNorm = 0.0;
            for (int i = 0; i < v.length; i++) {
                vPrev[i] = v[i];
                double a0 = a[i];
                v[i] = velocityAt(i, vPrev[i], a0, dt);
                a[i] = accelAt(i, a0, dt);
                j[i] = jerkAt(i, dt);
                aNorm += a[i] * a[i];
            }
            aNorm = Math.sqrt(aNorm);

            // The safety clamp: never above the limit by more than the factor, never above what
            // it already was. Re-integrate the velocity from the clamped acceleration so what is
            // stated and what is commanded stay one motion; the next tick re-plans.
            double aMax = Math.max(kClampFactor * aLim, aNow);
            clamped = aNorm >= aMax && aNorm > 0.0;
            if (clamped) {
                clampLatched = true;
                double scale = aMax / aNorm;
                for (int i = 0; i < v.length; i++) {
                    a[i] *= scale;
                    v[i] = vPrev[i] + a[i] * dt;
                    j[i] = 0.0;
                }
            }
        }

        /** Quartic Hermite from {@code (v, a)} now to {@code (vDes, 0, 0)} at {@code t}. */
        private void fit(double t) {
            double t2 = t * t, t3 = t2 * t, t4 = t3 * t;
            for (int i = 0; i < v.length; i++) {
                double delta = vDes[i] - v[i] - a[i] * t;
                double bt = -a[i] * t;
                c2[i] = (6.0 * delta - 3.0 * bt) / t2;
                c3[i] = (5.0 * bt - 8.0 * delta) / t3;
                c4[i] = (3.0 * delta - 2.0 * bt) / t4;
            }
        }

        /**
         * Whether the plan's peak |a| and |j| over {@code [0, t]} fit the given set. Per component
         * the extrema are exact: the jerk (a quadratic) is extremal where the snap vanishes, the
         * acceleration (a cubic) where the jerk vanishes. The components' peaks are summed in
         * quadrature, which is exact when they peak together (the common case: one direction) and
         * conservative otherwise.
         */
        private boolean peaksFit(double t, double aLim, double jLim) {
            double aSq = 0.0, jSq = 0.0;
            for (int i = 0; i < v.length; i++) {
                double jPk = Math.max(Math.abs(jerkAt(i, 0.0)), Math.abs(jerkAt(i, t)));
                double aPk = Math.max(Math.abs(a[i]), Math.abs(accelAt(i, a[i], t)));
                // snap 6c3 + 24c4 τ = 0
                if (c4[i] != 0.0) {
                    double ts = -c3[i] / (4.0 * c4[i]);
                    if (ts > 0.0 && ts < t) jPk = Math.max(jPk, Math.abs(jerkAt(i, ts)));
                }
                // jerk 12c4 τ² + 6c3 τ + 2c2 = 0
                double qa = 12.0 * c4[i], qb = 6.0 * c3[i], qc = 2.0 * c2[i];
                if (Math.abs(qa) > 1e-12 * Math.max(Math.abs(qb), Math.abs(qc))) {
                    double disc = qb * qb - 4.0 * qa * qc;
                    if (disc >= 0.0) {
                        double sq = Math.sqrt(disc);
                        for (double r :
                                new double[] {(-qb - sq) / (2.0 * qa), (-qb + sq) / (2.0 * qa)})
                            if (r > 0.0 && r < t)
                                aPk = Math.max(aPk, Math.abs(accelAt(i, a[i], r)));
                    }
                } else if (qb != 0.0) {
                    double r = -qc / qb;
                    if (r > 0.0 && r < t) aPk = Math.max(aPk, Math.abs(accelAt(i, a[i], r)));
                }
                aSq += aPk * aPk;
                jSq += jPk * jPk;
            }
            return aSq <= aLim * aLim && jSq <= jLim * jLim;
        }

        private double velocityAt(int i, double v0, double a0, double t) {
            double t2 = t * t;
            return v0 + a0 * t + c2[i] * t2 + c3[i] * t2 * t + c4[i] * t2 * t2;
        }

        private double accelAt(int i, double a0, double t) {
            return a0 + 2.0 * c2[i] * t + 3.0 * c3[i] * t * t + 4.0 * c4[i] * t * t * t;
        }

        private double jerkAt(int i, double t) {
            return 2.0 * c2[i] + 6.0 * c3[i] * t + 12.0 * c4[i] * t * t;
        }
    }
}
