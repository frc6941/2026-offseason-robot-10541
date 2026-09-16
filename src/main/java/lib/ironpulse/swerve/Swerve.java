package lib.ironpulse.swerve;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveBodyVelParamsNT;
import frc.robot.SwerveModuleParamsNT;
import frc.robot.SwerveTractionParamsNT;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import lib.ironpulse.limelight.Localizable;
import lib.ironpulse.utils.LoggedTracer;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

/**
 * Swerve drivetrain, and the drive-control topology contract.
 *
 * <p>Every drive command runs the same spine, and there is only one of it — the velocity loop lives
 * on the RIO and the motors take raw torque current:
 *
 * <pre>
 *   VRT → ChassisProfile (jerk-limited reference, states its acceleration)
 *       → setpoint generator (desaturate, steer coupling) → per module commanded state
 *   SwerveDriveComposer.compute → per module feedforward force → amps
 *   BodyVelocityEstimator.update → the chassis's own velocity, from the IMU (periodic, every tick)
 *   TractionGovernor.update → per wheel slip ratio against it: a scale, and the speed P sees
 *   I_i = FF_i + kP·(v_cmd,i − v_P,i) + kS·smooth-sign(v_cmd,i) + kV·v_cmd,i   (compose)
 *   scale_i · I_i → SwerveLimiter (headroom) ON THE WHOLE CURRENT
 *   module ← raw TorqueCurrentFOC(limited I_i)
 * </pre>
 *
 * <p>The point of composing on the RIO is that the limiter bounds the drivetrain's real draw by
 * construction — the P refill that a firmware velocity loop adds underneath a bounded feedforward
 * would never pass through the headroom law. Feedforward is source-agnostic: the model in teleop,
 * PathPlanner's torque current in auto. The acceleration the model works from is the profile's own,
 * every source alike; nothing on this spine differentiates a velocity.
 *
 * <p>Responsibilities are split so no class does two jobs: {@link ChassisProfile} shapes the
 * reference, {@link SwerveDriveComposer} is the control law — force model plus current sum — {@link
 * BodyVelocityEstimator} states where the chassis actually is going, {@link TractionGovernor} is
 * traction, per wheel, and {@link SwerveLimiter} is feasibility. This class only routes between
 * them and the hardware.
 */
public class Swerve extends SubsystemBase implements Localizable {
    // locks
    static final Lock odometryLock = new ReentrantLock();
    // config and io
    private final SwerveConfig config;
    private final List<SwerveModule> modules;
    private final ImuIO imuIO;
    // controller
    private final SwerveDriveKinematics kinematics;
    private final SwerveSetpointGenerator setpointGenerator;
    private final SwerveLimiter limiter;
    private final SwerveDriveComposer composer;
    private final TractionGovernor governor;
    private final BodyVelocityEstimator bodyVelocity;

    /**
     * This tick's BodyVel knobs, read once at the top of periodic. Read twice, a knob edited
     * between the two reads is logged as one value and used as another.
     */
    private BodyVelocityEstimator.Limits bodyLimits = bodyVelLimits();

    /** The same for the governor's knobs, read once at the top of the tick for the same reason. */
    private TractionGovernor.Limits tractionKnobs = tractionLimits();

    /** Reused buffer for the modules' measured rotor speed, which sets back EMF and duty. */
    private final double[] driveRotorRadPerSec;

    /** Reused buffers for the composition: feedforward amps, commanded speeds and headings. */
    private final double[] composeFfAmps;

    private final double[] composeCmdMps;
    private final Rotation2d[] composeHeadingsCmd;

    /**
     * This tick's measured module speeds and headings, filled once by {@link #refreshMeasuredState}
     * and read by both the body-velocity estimator and the composition.
     */
    private final double[] measuredMps;

    private final Rotation2d[] measuredHeadings;

    /**
     * The previous tick's composed drive current per module, before the governor's scale. The
     * governor reads its sign to tell a wheel the drivetrain is driving from one it is already
     * pulling back; this tick's is not composed yet, and composing it depends on what the governor
     * decides.
     */
    private final double[] composedAmpsPrev;

    /**
     * PathPlanner's own per module torque currents, in amperes. Zeroed whenever auto is not
     * driving, so the key does not hold stale auto values through teleop.
     */
    private final double[] ppTauAmps;

    // estimator
    private final SwerveDrivePoseEstimator3d poseEstimator;
    // precomputed
    private final List<Rotation2d> xLockAngles;
    public final ImuIOInputsAutoLogged imuIOInputs;
    private SwerveSetpoint setpointCurr;
    @Getter private Voltage previouslyAppliedVoltage;
    private MODE mode = MODE.VELOCITY;
    @Getter private Translation2d centerOfRotation = Translation2d.kZero;
    private DoubleSupplier totalCurrentSupplier = () -> Double.NaN;

    /** Optional measured whole-pack current, in supply amperes (e.g. PDH/PDP). */
    public void setTotalCurrentSupplier(DoubleSupplier supplier) {
        totalCurrentSupplier = Objects.requireNonNull(supplier);
    }

    public void setCenterOfRotation(Translation2d center) {
        centerOfRotation = center == null ? Translation2d.kZero : center;
    }

    public void resetCenterOfRotation() {
        centerOfRotation = Translation2d.kZero;
    }

    private ChassisSpeeds centered(ChassisSpeeds desired) {
        double w = desired.omegaRadiansPerSecond;
        return new ChassisSpeeds(
                desired.vxMetersPerSecond + w * centerOfRotation.getY(),
                desired.vyMetersPerSecond - w * centerOfRotation.getX(),
                w);
    }

    public double[] getDriveAppliedVolts() {
        double[] volts = new double[modules.size()];
        for (int i = 0; i < modules.size(); i++) volts[i] = modules.get(i).getDriveAppliedVolts();
        return volts;
    }

    private SwerveLimit profileLimit() {
        ChassisProfile.Limits limits = getProfileLimits();
        return SwerveLimit.builder()
                .maxLinearVelocity(MetersPerSecond.of(limits.maxLinearVelocity()))
                .maxSkidAcceleration(MetersPerSecondPerSecond.of(limits.maxLinearAccel()))
                .maxBrakeAcceleration(MetersPerSecondPerSecond.of(limits.maxBrakeAccel()))
                .maxAngularVelocity(RadiansPerSecond.of(limits.maxAngularVelocity()))
                .maxAngularAcceleration(RadiansPerSecondPerSecond.of(limits.maxAngularAccel()))
                .build();
    }

    public Swerve(SwerveConfig swerveConfig, ImuIO imuIO, SwerveModuleIO... moduleIOs) {
        swerveConfig.validateForceModel();
        this.config = swerveConfig;
        this.modules = new ArrayList<>(moduleIOs.length);
        if (config.moduleCount() != moduleIOs.length)
            throw new Error(
                    "Module count mismatch: " + config.moduleCount() + " vs " + moduleIOs.length);

        // ios
        this.imuIO = imuIO;
        this.imuIOInputs = new ImuIOInputsAutoLogged();
        for (int i = 0; i < config.moduleConfigs.length; i++)
            this.modules.add(i, new SwerveModule(config, config.moduleConfigs[i], moduleIOs[i]));
        SwerveModuleState[] states = new SwerveModuleState[config.moduleCount()];
        for (int i = 0; i < config.moduleCount(); i++)
            states[i] = modules.get(i).getSwerveModuleState();

        // kinematics, profile, and setpoint generator. The profile's velocity caps are bounded by
        // what the modules can run: the free speed, and that over the farthest module's lever arm
        // for pure rotation, so desaturation never rescales a profiled twist on its own.
        kinematics = new SwerveDriveKinematics(config.moduleLocations());
        double moduleVMax = config.defaultSwerveModuleLimit.maxDriveVelocity().in(MetersPerSecond);
        double leverArm = 0.0;
        for (var location : config.moduleLocations())
            leverArm = Math.max(leverArm, location.getNorm());
        setpointGenerator =
                SwerveSetpointGenerator.builder()
                        .kinematics(kinematics)
                        .profile(new ChassisProfile(moduleVMax, leverArm))
                        .moduleLimit(config.defaultSwerveModuleLimit)
                        .build();
        ChassisProfile.Limits limits = getProfileLimits();
        if (setpointGenerator.getProfile().capsBound())
            DriverStation.reportWarning(
                    "Params/Swerve/Profile velocity caps exceed what the modules can run; bound to"
                            + " maxLinearVelocity "
                            + limits.maxLinearVelocity()
                            + " m/s, maxAngularVelocity "
                            + limits.maxAngularVelocity()
                            + " rad/s (module free speed "
                            + moduleVMax
                            + " m/s, lever arm "
                            + leverArm
                            + " m)",
                    false);
        setpointCurr = new SwerveSetpoint(new ChassisSpeeds(), states, new ChassisAccel(0, 0, 0));
        limiter = new SwerveLimiter(config);
        composer = new SwerveDriveComposer(config);
        governor = new TractionGovernor(config.moduleLocations(), config.dtS);
        bodyVelocity = new BodyVelocityEstimator(config.moduleLocations(), config.dtS);
        driveRotorRadPerSec = new double[config.moduleCount()];
        composeFfAmps = new double[config.moduleCount()];
        composeCmdMps = new double[config.moduleCount()];
        composeHeadingsCmd = new Rotation2d[config.moduleCount()];
        measuredMps = new double[config.moduleCount()];
        measuredHeadings = new Rotation2d[config.moduleCount()];
        composedAmpsPrev = new double[config.moduleCount()];
        ppTauAmps = new double[config.moduleCount()];

        // estimator
        // Odometry state std-devs (x, y, z, angle) raised from the WPILib default 0.1 -> 0.15 on
        // the translation axes: this robot bumps field elements/other robots, wheels slip, and
        // wheel dead-reckoning walks the estimate off-field. Raising these tells the estimator to
        // trust odometry less (and to grow its uncertainty faster during a vision dropout), so
        // vision re-anchors the pose sooner after a slip. z/angle kept at 0.1. Vision std-devs here
        // are just an initial default -- every addVisionMeasurement() passes its own per-frame
        // std-devs (see LimelightSubsystem / LimeLightConfig Params/LL).
        poseEstimator =
                new SwerveDrivePoseEstimator3d(
                        kinematics,
                        new Rotation3d(),
                        getModulePositions(),
                        new Pose3d(),
                        VecBuilder.fill(0.15, 0.15, 0.1, 0.1),
                        VecBuilder.fill(0.9, 0.9, 0.9, 0.9));

        // precompute
        var moduleLocations = config.moduleLocations();
        xLockAngles = new ArrayList<>(config.moduleCount());
        for (int i = 0; i < config.moduleCount(); i++)
            xLockAngles.add(i, moduleLocations[i].getAngle());
    }

    // ------- Core Methods -------
    @Override
    public void periodic() {
        // Nothing that is a carried command may cross a disable: neither the logged current
        // split nor — the one that would actually kick the drivetrain — the profile's plan, which
        // would otherwise resume from a pre-disable velocity the chassis no longer has. Cleared
        // here because periodic runs while disabled, when the run methods do not.
        //
        // The body-velocity estimator is deliberately not in this list. It is a measurement, and
        // the stillest window the robot gets is exactly this one: clearing it every disabled tick
        // kept its rest counter at 1 and made the accelerometer bias unlearnable until the robot
        // had already been enabled and standing still, which on a boot-then-auto is never.
        if (RobotState.isDisabled()) {
            composer.reset();
            governor.reset();
            forgetComposedAmps();
            resetReference(getChassisSpeeds(), getModuleStates(), true);
        }

        // io updates
        odometryLock.lock();
        imuIOInputs.yawVelocityRadPerSecCmd = getChassisSpeeds().omegaRadiansPerSecond;
        imuIO.updateInputs(imuIOInputs);
        Logger.processInputs(config.name + "/IMU", imuIOInputs);
        modules.forEach(
                module -> {
                    module.updateInputs();
                    module.periodic();
                });

        // odom
        var swerveModulePositionsWithTime = getSampledModulePositions();
        var rotations = imuIOInputs.odometryRotations;
        var now = Timer.getTimestamp();
        for (int i = 0; i < swerveModulePositionsWithTime.size(); i++) {
            var positionWithTime = swerveModulePositionsWithTime.get(i);
            poseEstimator.updateWithTime(
                    now,
                    rotations[i], // FIXME: there's a discrepancy between Phoenix time and rio time.
                    // need to find
                    // the offset. this fix is temporary
                    positionWithTime.getSecond());
        }
        odometryLock.unlock();
        LoggedTracer.record(config.name + "/Inputs");

        bodyLimits = bodyVelLimits();
        tractionKnobs = tractionLimits();
        updateBodyVelocity();
        logBodyVel();

        // telemetry
        Logger.recordOutput(config.name + "/Mode", mode);
        Logger.recordOutput(config.name + "/ChassisSpeedCurr", getChassisSpeeds());
        Logger.recordOutput(config.name + "/SwerveModuleStateCurr", getModuleStates());
        Logger.recordOutput(config.name + "/DriveAppliedVolts", getDriveAppliedVolts());
        Logger.recordOutput(config.name + "/DriveTorqueCurrentAmps", getDriveTorqueCurrentAmps());
        Logger.recordOutput(config.name + "/DriveSupplyCurrentAmps", getDriveSupplyCurrentAmps());
        Logger.recordOutput(config.name + "/SwerveModuleStateCmd", setpointCurr.moduleStates());
        Logger.recordOutput(config.name + "/ChassisSpeedCmd", setpointCurr.chassisSpeeds());

        // Scalar mirrors make tracking logs readable without struct schema support.
        SwerveModuleState[] cmdStates = setpointCurr.moduleStates();
        SwerveModuleState[] curStates = getModuleStates();
        double[] cmdSpeeds = new double[cmdStates.length];
        double[] curSpeeds = new double[curStates.length];
        double[] speedError = new double[cmdStates.length];
        double[] cmdAngles = new double[cmdStates.length];
        double[] curAngles = new double[curStates.length];
        for (int i = 0; i < cmdStates.length; i++) {
            cmdSpeeds[i] = cmdStates[i].speedMetersPerSecond;
            curSpeeds[i] = curStates[i].speedMetersPerSecond;
            speedError[i] = cmdSpeeds[i] - curSpeeds[i];
            cmdAngles[i] = cmdStates[i].angle.getRadians();
            curAngles[i] = curStates[i].angle.getRadians();
        }
        Logger.recordOutput(config.name + "/Track/ModuleSpeedCmdMps", cmdSpeeds);
        Logger.recordOutput(config.name + "/Track/ModuleSpeedCurrMps", curSpeeds);
        Logger.recordOutput(config.name + "/Track/ModuleSpeedErrorMps", speedError);
        Logger.recordOutput(config.name + "/Track/ModuleAngleCmdRad", cmdAngles);
        Logger.recordOutput(config.name + "/Track/ModuleAngleCurrRad", curAngles);

        ChassisSpeeds cmdChassis = setpointCurr.chassisSpeeds();
        ChassisSpeeds curChassis = getChassisSpeeds();
        Logger.recordOutput(
                config.name + "/Track/ChassisCmdVxVyOmega",
                new double[] {
                    cmdChassis.vxMetersPerSecond,
                    cmdChassis.vyMetersPerSecond,
                    cmdChassis.omegaRadiansPerSecond
                });
        Logger.recordOutput(
                config.name + "/Track/ChassisCurrVxVyOmega",
                new double[] {
                    curChassis.vxMetersPerSecond,
                    curChassis.vyMetersPerSecond,
                    curChassis.omegaRadiansPerSecond
                });
        Logger.recordOutput(
                config.name + "/SwerveEstimatorPose", poseEstimator.getEstimatedPosition());

        logLiveParams();

        var wrench = getWrench();
        Logger.recordOutput(config.name + "/Wrench/FxN", wrench[0]);
        Logger.recordOutput(config.name + "/Wrench/FyN", wrench[1]);
        Logger.recordOutput(config.name + "/Wrench/TauNm", wrench[2]);
    }

    // -------- Run -------

    /**
     * Run a twist for the swerve drive.
     *
     * @param VRT the desired twist. note twist is expressed under the robot frame. if want to use
     *     field oriented drive,need to do frame transform elsewhere before pass in the command. The
     *     profile shapes it and states the acceleration the feedforward works from; no caller
     *     states one.
     */
    public void runTwist(ChassisSpeeds VRT) {
        mode = MODE.VELOCITY;
        setpointCurr =
                setpointGenerator.generate(centered(VRT), setpointCurr, yawDeltaRad(), config.dtS);
        composer.compute(setpointCurr);
        refreshPowerState();

        // Nothing from the auto path is live on this tick; zero it rather than leaving the last
        // auto values on the key through all of teleop.
        Arrays.fill(ppTauAmps, 0.0);

        // The model feedforward is one term of the composed current, not a request rider, so it is
        // always included — zero when the model faulted, which leaves the loop running on P and
        // friction alone rather than dropping the wheels.
        for (int i = 0; i < config.moduleCount(); i++)
            composeFfAmps[i] = limiter.statorAmpsForForce(composer.getForcesNewton()[i]);
        double[] commanded = composeAndLimit(composeFfAmps);
        logCompose(commanded);

        SwerveModuleState[] states = setpointCurr.moduleStates();
        for (int i = 0; i < config.moduleCount(); i++)
            modules.get(i)
                    .runStateCurrent(
                            states[i], Amps.of(commanded[i]), setpointGenerator.isSteerCoupled());
    }

    public void runTwistWithTorque(ChassisSpeeds VRT, Current[] tau) {
        if (tau == null || tau.length != config.moduleCount())
            throw new IllegalArgumentException("Torque current count must match module count");
        for (Current current : tau)
            if (current == null || !Double.isFinite(current.in(Amps)))
                throw new IllegalArgumentException("Torque currents must be finite");
        mode = MODE.VELOCITY;
        setpointCurr =
                setpointGenerator.generate(centered(VRT), setpointCurr, yawDeltaRad(), config.dtS);

        // The model still runs, so the log carries its answer next to the one actually used.
        composer.compute(setpointCurr);
        refreshPowerState();
        for (int i = 0; i < tau.length; i++) ppTauAmps[i] = tau[i].in(Amps);

        // Source-agnostic: PathPlanner's torque current is the feedforward term the composer builds
        // the loop around, exactly as the model feedforward is in teleop.
        double[] commanded = composeAndLimit(ppTauAmps);
        logCompose(commanded);

        SwerveModuleState[] states = setpointCurr.moduleStates();
        for (int i = 0; i < config.moduleCount(); i++)
            modules.get(i)
                    .runStateCurrent(
                            states[i], Amps.of(commanded[i]), setpointGenerator.isSteerCoupled());
    }

    /**
     * Mirrors the live tunables into the log every tick.
     *
     * <p>A bag that records what the robot did but not what it was configured to do is unreadable
     * after the fact: two A/B sessions were lost to exactly this, because the gains and toggles
     * being compared existed only in NetworkTables and never in the log. These are the values as
     * actually used — post-clamp, read the same way the control path reads them — so a bag answers
     * "what was this run" without anyone having to remember.
     *
     * <p>Logged from {@code periodic} rather than alongside the feedforward outputs so they are
     * present while the robot is disabled too, which is when the knobs get changed between runs.
     */
    private void logLiveParams() {
        Logger.recordOutput(
                config.name + "/Compose/params/scale", SwerveDriveComposer.feedforwardScale());
        Logger.recordOutput(
                config.name + "/Compose/params/pFilterHz", SwerveDriveComposer.pFilterHz());
        Logger.recordOutput(config.name + "/Compose/params/driveKP", driveKP());
        Logger.recordOutput(config.name + "/Compose/params/driveKS", driveKS());
        Logger.recordOutput(config.name + "/Compose/params/driveKV", driveKV());
        Logger.recordOutput(config.name + "/Limiter/params/enabled", SwerveLimiter.isEnabled());
        Logger.recordOutput(
                config.name + "/Limiter/params/iBudgetAmps", SwerveLimiter.iBudgetAmps());
        TractionGovernor.Limits traction = tractionKnobs;
        Logger.recordOutput(config.name + "/Traction/params/enabled", traction.enabled());
        Logger.recordOutput(config.name + "/Traction/params/lambdaStar", traction.lambdaStar());
        Logger.recordOutput(config.name + "/Traction/params/dvHi", traction.dvHi());
        Logger.recordOutput(config.name + "/Traction/params/kP", traction.kP());
        Logger.recordOutput(config.name + "/Traction/params/scaleMin", traction.scaleMin());
        Logger.recordOutput(config.name + "/Traction/params/releaseS", traction.releaseS());
        Logger.recordOutput(config.name + "/Traction/params/vFloor", traction.vFloor());
        Logger.recordOutput(config.name + "/Traction/params/cmAccel", traction.cmAccel());
        Logger.recordOutput(config.name + "/Traction/params/leadS", traction.leadS());
        BodyVelocityEstimator.Limits body = bodyLimits;
        Logger.recordOutput(config.name + "/BodyVel/params/gateMps", body.gateMps());
        Logger.recordOutput(config.name + "/BodyVel/params/seedTauS", body.seedTauS());
        Logger.recordOutput(config.name + "/BodyVel/params/seedRateCap", body.seedRateCap());
        Logger.recordOutput(config.name + "/BodyVel/params/biasTauS", body.biasTauS());
        Logger.recordOutput(config.name + "/BodyVel/params/imuStaleS", body.imuStaleS());
        Logger.recordOutput(config.name + "/BodyVel/params/plausibleMps", body.plausibleMps());
        ChassisProfile.Limits profile = getProfileLimits();
        Logger.recordOutput(
                config.name + "/Profile/params/capsBound",
                setpointGenerator.getProfile().capsBound());
        Logger.recordOutput(
                config.name + "/Profile/params/maxLinearVelocity", profile.maxLinearVelocity());
        Logger.recordOutput(
                config.name + "/Profile/params/maxLinearAccel", profile.maxLinearAccel());
        Logger.recordOutput(config.name + "/Profile/params/maxLinearJerk", profile.maxLinearJerk());
        Logger.recordOutput(config.name + "/Profile/params/maxBrakeAccel", profile.maxBrakeAccel());
        Logger.recordOutput(config.name + "/Profile/params/maxBrakeJerk", profile.maxBrakeJerk());
        Logger.recordOutput(
                config.name + "/Profile/params/maxAngularVelocity", profile.maxAngularVelocity());
        Logger.recordOutput(
                config.name + "/Profile/params/maxAngularAccel", profile.maxAngularAccel());
        Logger.recordOutput(
                config.name + "/Profile/params/maxAngularJerk", profile.maxAngularJerk());
    }

    /**
     * Seeds the reference at a measured twist: the profile at {@code (v, 0, 0)} with no plan, and
     * the previous setpoint the generator interpolates from at the same twist and the modules'
     * present angles. The disable path and the X-lock also clear the profile's latched clamp fault;
     * a plain stop re-seeds and leaves it visible until then.
     */
    private void resetReference(
            ChassisSpeeds speeds, SwerveModuleState[] states, boolean clearFault) {
        setpointGenerator.reset(speeds, clearFault);
        setpointCurr = new SwerveSetpoint(speeds, states, new ChassisAccel(0.0, 0.0, 0.0));
        yawPrevRad = Double.NaN;
    }

    /** The gyro yaw at the previous reference tick; NaN until one has run since a reset. */
    private double yawPrevRad = Double.NaN;

    /**
     * How far the chassis turned since the previous reference tick, from the gyro — the rotation
     * the profile's carried vectors are re-expressed through. Zero on the first tick after a reset,
     * so a re-enable never spins the reference by the yaw accumulated while disabled.
     */
    private double yawDeltaRad() {
        double yaw = imuIOInputs.yawPosition.getRadians();
        double d = Double.isNaN(yawPrevRad) ? 0.0 : yaw - yawPrevRad;
        yawPrevRad = yaw;
        return Math.atan2(Math.sin(d), Math.cos(d));
    }

    /** Refresh measured drive supply current, rotor speed, and optional whole-pack draw. */
    private void refreshPowerState() {
        double driveAmps = 0.0;
        for (int i = 0; i < modules.size(); i++) {
            driveAmps += Math.abs(modules.get(i).getDriveSupplyCurrentAmpere());
            driveRotorRadPerSec[i] = modules.get(i).getDriveRotorVelocityRadPerSec();
        }
        limiter.updatePower(totalCurrentSupplier.getAsDouble(), driveAmps);
    }

    /**
     * Compose, apply traction scales, then budget the entire current at roboRIO battery voltage.
     */
    private double[] composeAndLimit(double[] feedforwardAmps) {
        SwerveModuleState[] cmd = setpointCurr.moduleStates();
        refreshMeasuredState();
        refreshCommandedHeadings();
        for (int i = 0; i < modules.size(); i++) composeCmdMps[i] = cmd[i].speedMetersPerSecond;
        governor.update(
                measuredMps,
                measuredHeadings,
                composeHeadingsCmd,
                composedAmpsPrev,
                setpointGenerator.isSteerCoupled(),
                getChassisSpeeds(),
                new TractionGovernor.Reference(
                        bodyVelocity.vx(),
                        bodyVelocity.vy(),
                        bodyVelocity.ax(),
                        bodyVelocity.ay(),
                        imuIOInputs.yawVelocityRadPerSec,
                        bodyVelocity.valid()),
                tractionKnobs);
        double[] total =
                composer.compose(
                        feedforwardAmps,
                        composeCmdMps,
                        governor.measuredForP(),
                        driveKP(),
                        driveKS(),
                        driveKV());
        // Only the velocity path composes, so every path that bypasses it has to say so — see
        // forgetComposedAmps — or the veto keeps reading a sign from a tick that no longer applies.
        System.arraycopy(total, 0, composedAmpsPrev, 0, total.length);
        double[] scale = governor.scale();
        for (int i = 0; i < modules.size(); i++) total[i] *= scale[i];
        return limiter.applyComposedAmps(
                total, driveRotorRadPerSec, RobotController.getBatteryVoltage());
    }

    /** The governor's live knobs, read from {@code Params/Swerve/Traction} and bounded. */
    private static TractionGovernor.Limits tractionLimits() {
        return new TractionGovernor.Limits(
                SwerveTractionParamsNT.enabled.getValue(),
                SwerveLimiter.bounded(
                        SwerveTractionParamsNT.lambdaStar.getValue(),
                        TractionGovernor.kDefaultLambdaStar,
                        0.01,
                        1.0),
                SwerveLimiter.bounded(
                        SwerveTractionParamsNT.dvHi.getValue(),
                        TractionGovernor.kDefaultDvHi,
                        0.05,
                        3.0),
                SwerveLimiter.bounded(
                        SwerveTractionParamsNT.kP.getValue(),
                        TractionGovernor.kDefaultKP,
                        0.0,
                        50.0),
                SwerveLimiter.bounded(
                        SwerveTractionParamsNT.scaleMin.getValue(),
                        TractionGovernor.kDefaultScaleMin,
                        0.0,
                        1.0),
                SwerveLimiter.bounded(
                        SwerveTractionParamsNT.releaseS.getValue(),
                        TractionGovernor.kDefaultReleaseS,
                        0.01,
                        1.0),
                SwerveLimiter.bounded(
                        SwerveTractionParamsNT.vFloor.getValue(),
                        TractionGovernor.kDefaultVFloor,
                        0.01,
                        2.0),
                SwerveLimiter.bounded(
                        SwerveTractionParamsNT.cmAccel.getValue(),
                        TractionGovernor.kDefaultCmAccel,
                        5.0,
                        100.0),
                SwerveLimiter.bounded(
                        SwerveTractionParamsNT.leadS.getValue(),
                        TractionGovernor.kDefaultLeadS,
                        0.0,
                        0.1));
    }

    /**
     * The composer's live drive gains, bounded the way every other live gain on this spine is.
     *
     * <p>A NaN reaching the composer comes back out as a NaN ampere, and from there poisons the
     * sign the traction veto reads and whatever the limiter made of it, without anything reporting
     * a fault. Read through here by the composer, the modules' own Slot0 configuration and the log
     * line alike, so the bag cannot show a gain the drivetrain did not actually use. Fallbacks are
     * {@code Params/SwerveModule/Drive}'s own compiled defaults.
     */
    static double driveKP() {
        return SwerveLimiter.bounded(SwerveModuleParamsNT.Drive.kP.getValue(), 6.0, 0.0, 100.0);
    }

    static double driveKS() {
        return SwerveLimiter.bounded(SwerveModuleParamsNT.Drive.kS.getValue(), 0.0, 0.0, 60.0);
    }

    static double driveKV() {
        return SwerveLimiter.bounded(SwerveModuleParamsNT.Drive.kV.getValue(), 0.136, 0.0, 20.0);
    }

    /** The estimator's live knobs, read from {@code Params/Swerve/BodyVel} and bounded. */
    static BodyVelocityEstimator.Limits bodyVelLimits() {
        return new BodyVelocityEstimator.Limits(
                SwerveLimiter.bounded(
                        SwerveBodyVelParamsNT.gateMps.getValue(),
                        BodyVelocityEstimator.kDefaultGateMps,
                        0.05,
                        5.0),
                SwerveLimiter.bounded(
                        SwerveBodyVelParamsNT.seedTauS.getValue(),
                        BodyVelocityEstimator.kDefaultSeedTauS,
                        0.005,
                        2.0),
                SwerveLimiter.bounded(
                        SwerveBodyVelParamsNT.seedRateCap.getValue(),
                        BodyVelocityEstimator.kDefaultSeedRateCap,
                        0.1,
                        50.0),
                SwerveLimiter.bounded(
                        SwerveBodyVelParamsNT.biasTauS.getValue(),
                        BodyVelocityEstimator.kDefaultBiasTauS,
                        0.1,
                        60.0),
                SwerveLimiter.bounded(
                        SwerveBodyVelParamsNT.imuStaleS.getValue(),
                        BodyVelocityEstimator.kDefaultImuStaleS,
                        0.005,
                        1.0),
                SwerveLimiter.bounded(
                        SwerveBodyVelParamsNT.plausibleMps.getValue(),
                        BodyVelocityEstimator.kDefaultPlausibleMps,
                        // Never under the speed the profile is allowed to ask for: a bound below
                        // that would be throwing the state away on an ordinary fast drive.
                        ChassisProfile.kDefaultMaxLinearVelocity,
                        20.0));
    }

    /**
     * Drops the remembered per module current, for the paths that command the modules without
     * composing one.
     *
     * <p>Both the governor's torque-signed release and the estimator's snap veto read the sign of
     * the previous tick's composed current, which is only written on the velocity path. Left alone
     * across a stop, an X-lock or a raw-voltage run they would go on reading a sign from a tick
     * whose command no longer applies. Zero means "no torque commanded", which is exactly what is
     * true here, and both laws already treat that as having no side.
     */
    private void forgetComposedAmps() {
        Arrays.fill(composedAmpsPrev, 0.0);
    }

    /**
     * This tick's commanded module headings, into the shared buffer.
     *
     * <p>Read from {@code setpointCurr}, so in periodic it is the heading the last drive tick
     * commanded — one tick old, against a 10° gate and a three-tick steer latch, which is inside
     * the noise of both.
     */
    private void refreshCommandedHeadings() {
        SwerveModuleState[] cmd = setpointCurr.moduleStates();
        for (int i = 0; i < modules.size(); i++) composeHeadingsCmd[i] = cmd[i].angle;
    }

    /** This tick's measured module speeds and headings, into the shared buffers. */
    private void refreshMeasuredState() {
        SwerveModuleState[] measured = getModuleStates();
        for (int i = 0; i < modules.size(); i++) {
            measuredMps[i] = measured[i].speedMetersPerSecond;
            measuredHeadings[i] = measured[i].angle;
        }
    }

    /**
     * The chassis's own velocity for this tick, from the IMU and the wheels.
     *
     * <p>Runs in periodic rather than on a drive path so the reference is continuous: the governor
     * reads a state integrated on every tick of the run, not only the ticks a twist was commanded.
     */
    private void updateBodyVelocity() {
        refreshMeasuredState();
        refreshCommandedHeadings();
        bodyVelocity.update(
                getChassisSpeeds(),
                measuredMps,
                measuredHeadings,
                composeHeadingsCmd,
                composedAmpsPrev,
                setpointGenerator.isSteerCoupled(),
                new BodyVelocityEstimator.Imu(
                        imuIOInputs.connected,
                        imuIOInputs.accelLatencyS,
                        imuIOInputs.accelXMps2,
                        imuIOInputs.accelYMps2,
                        imuIOInputs.pitchPosition.getRadians(),
                        imuIOInputs.rollPosition.getRadians(),
                        imuIOInputs.yawVelocityRadPerSec),
                bodyLimits);
    }

    /** Per wheel: slip ratio, reference, and what the governor did about it. */
    private void logTraction() {
        Logger.recordOutput(config.name + "/Traction/lambda", governor.lambda());
        Logger.recordOutput(config.name + "/Traction/excess", governor.excess());
        Logger.recordOutput(config.name + "/Traction/vRef", governor.reference());
        Logger.recordOutput(config.name + "/Traction/engaged", governor.engaged());
        Logger.recordOutput(config.name + "/Traction/scale", governor.scale());
        Logger.recordOutput(config.name + "/Traction/trigger", governor.trigger());
        Logger.recordOutput(config.name + "/Traction/judged", governor.judged());
        Logger.recordOutput(config.name + "/Traction/cmTrigger", governor.cmTrigger());
        Logger.recordOutput(config.name + "/Traction/refValid", governor.refValid());
        Logger.recordOutput(config.name + "/Traction/finite", governor.inputsFinite());
    }

    /** The reference every wheel is judged against, and what it is currently worth. */
    private void logBodyVel() {
        Logger.recordOutput(config.name + "/BodyVel/vx", bodyVelocity.vx());
        Logger.recordOutput(config.name + "/BodyVel/vy", bodyVelocity.vy());
        Logger.recordOutput(config.name + "/BodyVel/ax", bodyVelocity.ax());
        Logger.recordOutput(config.name + "/BodyVel/ay", bodyVelocity.ay());
        Logger.recordOutput(config.name + "/BodyVel/valid", bodyVelocity.valid());
        Logger.recordOutput(config.name + "/BodyVel/fed", bodyVelocity.fed());
        Logger.recordOutput(config.name + "/BodyVel/feeding", bodyVelocity.feeding());
        Logger.recordOutput(config.name + "/BodyVel/judgeable", bodyVelocity.judgeableWheels());
        Logger.recordOutput(config.name + "/BodyVel/snaps", bodyVelocity.snaps());
        Logger.recordOutput(
                config.name + "/BodyVel/plausibilitySnaps", bodyVelocity.plausibilitySnaps());
        Logger.recordOutput(config.name + "/BodyVel/horizonS", bodyVelocity.horizonS());
        Logger.recordOutput(config.name + "/BodyVel/innovation", bodyVelocity.innovation());
        Logger.recordOutput(config.name + "/BodyVel/biasX", bodyVelocity.biasX());
        Logger.recordOutput(config.name + "/BodyVel/biasY", bodyVelocity.biasY());
    }

    private void logLimiter() {
        Logger.recordOutput(config.name + "/Limiter/iBudget", limiter.getIBudget());
        Logger.recordOutput(config.name + "/Limiter/driveHeadroom", limiter.getDriveHeadroom());
        Logger.recordOutput(config.name + "/Limiter/wrenchScale", limiter.getWrenchScale());
        Logger.recordOutput(
                config.name + "/Limiter/statorDemandAmps", limiter.getStatorDemandAmps());
        Logger.recordOutput(
                config.name + "/Limiter/supplyDemandAmps", limiter.getSupplyDemandAmps());
        Logger.recordOutput(config.name + "/Limiter/headroomApplied", limiter.isHeadroomApplied());
    }

    /**
     * The control law's whole answer for this tick: the model's half, the per module split of the
     * composed current — how much of each wheel's amps is feedforward, P refill, friction — and
     * what was finally commanded after the limiter.
     */
    private void logCompose(double[] ampsCmd) {
        Logger.recordOutput(config.name + "/Compose/aCmd", composer.getAccelerationCmd());
        Logger.recordOutput(config.name + "/Compose/wrench", composer.getWrench());
        Logger.recordOutput(config.name + "/Compose/forcesN", composer.getForcesNewton());
        Logger.recordOutput(config.name + "/Compose/fault", composer.isFault());
        Logger.recordOutput(config.name + "/Compose/ffAmps", composer.getFfAmps());
        Logger.recordOutput(config.name + "/Compose/pAmps", composer.getPAmps());
        Logger.recordOutput(config.name + "/Compose/pClamped", composer.isPClamped());
        Logger.recordOutput(config.name + "/Compose/frictionAmps", composer.getFrictionAmps());
        Logger.recordOutput(config.name + "/Compose/ampsCmd", ampsCmd);
        Logger.recordOutput(config.name + "/Compose/ppTauAmps", ppTauAmps);
        logProfile();
        logTraction();
        logLimiter();
    }

    /** The reference the profile planned this tick: state, horizons, and whether the clamp bit. */
    private void logProfile() {
        ChassisProfile profile = setpointGenerator.getProfile();
        ChassisProfile.State s = profile.state();
        Logger.recordOutput(config.name + "/Profile/v", new double[] {s.vx(), s.vy(), s.omega()});
        Logger.recordOutput(config.name + "/Profile/a", new double[] {s.ax(), s.ay(), s.alpha()});
        Logger.recordOutput(config.name + "/Profile/j", new double[] {s.jx(), s.jy(), s.zeta()});
        Logger.recordOutput(config.name + "/Profile/T", profile.horizons());
        Logger.recordOutput(config.name + "/Profile/clampEngaged", profile.clampEngaged());
        Logger.recordOutput(config.name + "/Profile/clampLatched", profile.clampLatched());
        Logger.recordOutput(
                config.name + "/Profile/steerCoupled", setpointGenerator.isSteerCoupled());
        Logger.recordOutput(config.name + "/Profile/desatScale", setpointGenerator.getDesatScale());
    }

    public void runVoltage(Voltage voltage) {
        mode = MODE.VOLTAGE;
        previouslyAppliedVoltage = voltage;
        forgetComposedAmps();
        for (int i = 0; i < config.moduleCount(); i++) modules.get(i).runDriveVoltage(voltage);
    }

    /**
     * 0 V on the drives, and the reference re-seeded at the measured twist so whoever drives next
     * plans from where the chassis actually is, not from the twist it was last asked for.
     */
    public void runStop() {
        runVoltage(Volt.of(0.0));
        resetReference(getChassisSpeeds(), getModuleStates(), false);
    }

    public void runStopAndLock() {
        mode = MODE.VELOCITY;
        forgetComposedAmps();
        SwerveModuleState[] lockStates = new SwerveModuleState[config.moduleCount()];
        for (int i = 0; i < config.moduleCount(); i++) {
            lockStates[i] = new SwerveModuleState(0.0, xLockAngles.get(i));
            modules.get(i).runStateForced(lockStates[i]);
        }
        kinematics.resetHeadings(xLockAngles.toArray(new Rotation2d[0]));
        resetReference(new ChassisSpeeds(), lockStates, true);
    }

    // ------- Getters -------
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.size()];
        for (int i = 0; i < modules.size(); i++) states[i] = modules.get(i).getSwerveModuleState();
        return states;
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[modules.size()];
        for (int i = 0; i < modules.size(); i++)
            states[i] = modules.get(i).getSwerveModulePosition();
        return states;
    }

    /** Drive-motor torque (stator) current (amps) for each module, indexed by module order. */
    public double[] getDriveTorqueCurrentAmps() {
        double[] amps = new double[modules.size()];
        for (int i = 0; i < modules.size(); i++)
            amps[i] = modules.get(i).getDriveTorqueCurrentAmpere();
        return amps;
    }

    /** Drive-motor supply current (amps) for each module, indexed by module order. */
    public double[] getDriveSupplyCurrentAmps() {
        double[] amps = new double[modules.size()];
        for (int i = 0; i < modules.size(); i++)
            amps[i] = modules.get(i).getDriveSupplyCurrentAmpere();
        return amps;
    }

    public List<Pair<Double, SwerveModulePosition[]>> getSampledModulePositions() {
        double[] timestamps = imuIOInputs.odometryYawTimestamps;
        int moduleCount = modules.size();

        // cache each module’s sampled positions array
        List<SwerveModulePosition[]> samplesByModule =
                modules.stream().map(SwerveModule::getSampledSwerveModulePositions).toList();

        // The IMU and each module drain their own high-frequency odometry queues, so on a given
        // loop they can report different sample counts (module positions are sized min(drive,
        // steer); the IMU timestamps come from the Pigeon queue). Process only the common prefix —
        // the min across the timestamps and every module — so a ragged tail can't index past a
        // shorter array (this was throwing ArrayIndexOutOfBoundsException here). Any extra samples
        // from a longer queue are dropped for this cycle, which is harmless for pose estimation.
        int sampleCount = Math.min(timestamps.length, imuIOInputs.odometryRotations.length);
        for (SwerveModulePosition[] moduleSamples : samplesByModule)
            sampleCount = Math.min(sampleCount, moduleSamples.length);

        List<Pair<Double, SwerveModulePosition[]>> result = new ArrayList<>(sampleCount);
        for (int sampleIdx = 0; sampleIdx < sampleCount; sampleIdx++) {
            // build the array of positions at this timestamp
            SwerveModulePosition[] positionsAtTime = new SwerveModulePosition[moduleCount];
            for (int moduleIdx = 0; moduleIdx < moduleCount; moduleIdx++)
                positionsAtTime[moduleIdx] = samplesByModule.get(moduleIdx)[sampleIdx];
            result.add(new Pair<>(timestamps[sampleIdx], positionsAtTime));
        }

        return result;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getChassisSpeedsCmd() {
        if (mode != MODE.VELOCITY) return new ChassisSpeeds();
        return setpointCurr.chassisSpeeds();
    }

    /**
     * The acceleration the current setpoint was planned with — what the feedforward was built from.
     */
    public ChassisAccel getChassisAccelCmd() {
        if (mode != MODE.VELOCITY) return new ChassisAccel(0.0, 0.0, 0.0);
        return setpointCurr.accel();
    }

    /** The profile's live, physically bounded limits — the caps the sources should scale to. */
    public ChassisProfile.Limits getProfileLimits() {
        return setpointGenerator.getProfile().limits();
    }

    /** The reference shaper itself, for the spine tests in this package. */
    ChassisProfile profile() {
        return setpointGenerator.getProfile();
    }

    /** The traction governor (package: spine tests). */
    TractionGovernor traction() {
        return governor;
    }

    /** The body-velocity reference the governor judges against (package: spine tests). */
    BodyVelocityEstimator bodyVelocity() {
        return bodyVelocity;
    }

    /** Whether modules still turning withheld part of this tick's step (package: spine tests). */
    boolean steerCoupled() {
        return setpointGenerator.isSteerCoupled();
    }

    /** This tick's desaturation factor (package: spine tests). */
    double desatScale() {
        return setpointGenerator.getDesatScale();
    }

    /**
     * Returns the chassis wrench as {@code [Fx (N), Fy (N), τ (N·m)]} in robot frame.
     *
     * <p>Computed by abusing {@link SwerveDriveKinematics#toChassisSpeeds}: each module's wheel
     * force (torque-current × Kt × gear-ratio / wheel-radius) is fed in as a speed, so the
     * kinematics least-squares projection maps wheel forces onto the chassis wrench space.
     */
    public double[] getWrench() {
        double wheelRadius = config.wheelDiameter.in(Meter) * 0.5;
        SwerveModuleState[] forceStates = new SwerveModuleState[modules.size()];
        for (int i = 0; i < modules.size(); i++) {
            double force =
                    modules.get(i).getDriveTorqueCurrentAmpere()
                            * config.driveMotorKt
                            * config.driveGearRatio
                            / wheelRadius;
            forceStates[i] =
                    new SwerveModuleState(
                            force, new Rotation2d(modules.get(i).getSteerAngle().in(Radian)));
        }
        ChassisSpeeds w = kinematics.toChassisSpeeds(forceStates);
        return new double[] {w.vxMetersPerSecond, w.vyMetersPerSecond, w.omegaRadiansPerSecond};
    }

    public Pose3d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetEstimatedPose(Pose3d pose) {
        odometryLock.lock();
        // Reset IMU hardware to match the new pose rotation
        double newYawDegrees = pose.getRotation().toRotation2d().getDegrees();
        imuIO.setYawDeg(newYawDegrees);
        // Reset pose estimator with the new pose
        // Use the new rotation from pose parameter, not stale imuIOInputs.odometryRotations[0]
        Rotation3d newRotation = pose.getRotation();
        poseEstimator.resetPosition(newRotation, getModulePositions(), pose);
        odometryLock.unlock();
    }

    public Optional<Pose3d> getEstimatedPoseAt(Time time) {
        return poseEstimator.sampleAt(time.in(Seconds));
    }

    @Override
    public void addVisionMeasurement(
            Pose3d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N4, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(
                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    @Override
    public double getIMUYaw() {
        return poseEstimator.getEstimatedPosition().getRotation().toRotation2d().getDegrees();
    }

    public double getYawVelocityRadPerSec() {
        return imuIOInputs.yawVelocityRadPerSec;
    }

    public double getPitchVelocityRadPerSec() {
        return imuIOInputs.pitchVelocityRadPerSec;
    }

    public double getPitchPosRad() {
        return imuIOInputs.pitchPosition.getRadians();
    }

    // ------- Configurations -------
    /**
     * Sets the drive motor neutral mode on every module: true = brake, false = coast. Steer motors
     * always stay in brake so the modules hold their angle. Each call reapplies the motor config
     * over CAN, so only call this on mode transitions, not periodically.
     */
    public void setDriveBrake(boolean isBrake) {
        modules.forEach(module -> module.setDriveBrake(isBrake));
    }

    public SwerveLimit getSwerveLimit() {
        return profileLimit();
    }

    public void setSwerveLimit(SwerveLimit limit) {
        setpointGenerator.getProfile().setLimitOverride(limit);
    }

    public void setSwerveLimitDefault() {
        setpointGenerator.getProfile().setLimitOverride(null);
    }

    public SwerveModuleLimit getSwerveModuleLimit() {
        return setpointGenerator.getModuleLimit();
    }

    public void setSwerveModuleLimit(SwerveModuleLimit limit) {
        setpointGenerator.setModuleLimit(limit);
    }

    public void setSwerveModuleLimitDefault() {
        setpointGenerator.setModuleLimit(config.defaultSwerveModuleLimit);
    }

    @Override
    public void setIMUYaw(double yaw) {
        imuIO.setYawDeg(yaw);
    }

    public enum MODE {
        VELOCITY,
        VOLTAGE
    }
}
