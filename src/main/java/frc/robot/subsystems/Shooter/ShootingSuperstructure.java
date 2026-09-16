package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.RobotStateRecorder;
import frc.robot.commands.AutoAimCommand;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Coordinates a drum-shooter shot: flywheel speed + hood angle + feed gating, driven by distance to
 * the hub via {@link ShotCalculator}.
 *
 * <p>Unlike a turret robot, yaw is actuated by the chassis (see {@link AutoAimCommand}). This
 * superstructure deliberately does NOT own swerve — it only reads the robot pose to compute
 * distance and heading error. Compose {@link #aimAndShoot()} in parallel with an {@link
 * AutoAimCommand} so the drivetrain handles yaw while this handles hood + flywheel + feed.
 *
 * <p>Everything derived from the robot pose (distance, shot solution, aim heading + rate) is
 * computed ONCE per loop in {@link #periodic()} and cached; the public getters and command
 * suppliers read the cache. Reading the pose re-queries the transform buffer (TreeMap lookup +
 * quaternion inverse) and solving re-builds the interpolation tables, so re-deriving per caller
 * used to run that chain several times per loop while shooting.
 */
public class ShootingSuperstructure extends SubsystemBase {
    private static final String MANUAL_OVERRIDE_KEY = "Shooter Tuning/Manual Override";
    private static final String MANUAL_HOOD_ANGLE_KEY = "Shooter Tuning/Hood Angle Deg";
    private static final String MANUAL_FLYWHEEL_RPS_KEY = "Shooter Tuning/Flywheel RPS";
    private static final double FEED_DELAY_AFTER_UPPER_READY_SECONDS = 0.1;

    // Below this magnitude the aim-heading rate is treated as zero. A stationary robot aimed at a
    // stationary hub has a true rate of 0; anything left after the moving-average filter is just
    // differentiated pose (vision) noise. Feeding that residual to AutoAim's omega feedforward
    // makes the chassis dither and scrubs the modules (reads as translational jitter), so we
    // floor it here.
    private static final double AIM_RATE_DEADBAND_RAD_S = 0.05;

    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooterUpper;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooterLower;
    private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood;
    private final HopperSubsystem hopper;
    private final ShotCalculator calculator = new ShotCalculator();

    // Per-loop pose-derived cache, refreshed in periodic().
    private double cachedDistanceMeters;
    private ShotSolution cachedSolution;
    private Rotation2d cachedAimHeading;
    private boolean cachedHeadingAtGoal;
    private Rotation2d lastAimHeading;
    private double lastAimTimestampSec = Double.NaN;
    private double aimRate;
    private final LinearFilter aimRateFilter =
            LinearFilter.movingAverage((int) (0.1 / RobotConstants.LOOPER_DT));

    public ShootingSuperstructure(
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooterUpper,
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooterLower,
            PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood,
            HopperSubsystem hopper) {
        this.shooterUpper = shooterUpper;
        this.shooterLower = shooterLower;
        this.hood = hood;
        this.hopper = hopper;

        Pose2d pose = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
        cachedDistanceMeters = AutoAimCommand.getDistanceToTarget(pose.getTranslation());
        cachedSolution = solutionForDistance(cachedDistanceMeters);
        cachedAimHeading = AutoAimCommand.getShooterAimHeading(pose);

        SmartDashboard.setDefaultBoolean(MANUAL_OVERRIDE_KEY, false);
        SmartDashboard.setDefaultNumber(MANUAL_HOOD_ANGLE_KEY, 10.0);
        SmartDashboard.setDefaultNumber(MANUAL_FLYWHEEL_RPS_KEY, 55.0);
        if (!RobotConstants.ENABLE_NT_PARAMS) {
            SmartDashboard.putBoolean(MANUAL_OVERRIDE_KEY, false);
        }
    }

    // ---------- per-loop cached state ----------

    /** Horizontal distance from the robot to the (alliance-flipped) target, in meters. */
    public double distanceToTarget() {
        return cachedDistanceMeters;
    }

    /** The stationary shot solution (hood angle + flywheel speed) for the current range. */
    public ShotSolution currentSolution() {
        return cachedSolution;
    }

    public Rotation2d aimHeading() {
        return cachedAimHeading;
    }

    public double aimHeadingRateRadPerSec() {
        return aimRate;
    }

    /** True when the shooter is pointed at the target within the configured heading tolerance. */
    public boolean headingAtGoal() {
        return cachedHeadingAtGoal;
    }

    /** All three shot DOFs satisfied: chassis aimed, hood at angle, flywheel up to speed. */
    public boolean readyToShoot() {
        return headingAtGoal() && hoodAtGoal() && shooterAtGoal();
    }

    public boolean shooterAtGoal() {
        return shooterUpper.velocityAtGoal() && shooterLower.velocityAtGoal();
    }

    public boolean hoodAtGoal() {
        return hood.positionAtGoal();
    }

    public boolean isShooterActive() {
        return shooterUpper.getCurrSetpoint().in(RotationsPerSecond)
                        > ShooterUpperParamsNT.idleRPS.getValue() + 1.0
                || shooterLower.getCurrSetpoint().in(RotationsPerSecond)
                        > ShooterLowerParamsNT.idleRPS.getValue() + 1.0;
    }

    private ShotSolution solutionForDistance(double distanceMeters) {
        if (manualOverrideEnabled()) {
            Angle hoodAngle =
                    clampHoodAngle(
                            Degrees.of(SmartDashboard.getNumber(MANUAL_HOOD_ANGLE_KEY, 10.0)));
            AngularVelocity flywheelSpeed =
                    RotationsPerSecond.of(
                            Math.max(0.0, SmartDashboard.getNumber(MANUAL_FLYWHEEL_RPS_KEY, 55.0)));
            return new ShotSolution(hoodAngle, flywheelSpeed);
        }
        return calculator.solve(distanceMeters);
    }

    private boolean manualOverrideEnabled() {
        return RobotConstants.ENABLE_NT_PARAMS
                && SmartDashboard.getBoolean(MANUAL_OVERRIDE_KEY, false);
    }

    private Angle clampHoodAngle(Angle angle) {
        return Degrees.of(
                MathUtil.clamp(
                        angle.in(Degrees),
                        ShooterConfig.HOOD_MIN_ANGLE.in(Degrees),
                        ShooterConfig.HOOD_MAX_ANGLE.in(Degrees)));
    }

    private AngularVelocity lowerSpeedFor(AngularVelocity upperSpeed) {
        return RotationsPerSecond.of(
                upperSpeed.in(RotationsPerSecond) * calculator.lowerShooterSpeedScale());
    }

    // ---------- shooter/hood primitives ----------

    public void configureDefaultCommands() {
        shooterUpper.setDefaultCommand(runUpperIdle());
        shooterLower.setDefaultCommand(runLowerIdle());
        hood.setDefaultCommand(hood.runMotionMagic(ShooterConfig.HOOD_STOW_ANGLE));
    }

    private Command runUpperIdle() {
        return shooterUpper.runVelVolt(
                () -> RotationsPerSecond.of(ShooterUpperParamsNT.idleRPS.getValue()));
    }

    private Command runLowerIdle() {
        return shooterLower.runVelVolt(
                () -> RotationsPerSecond.of(ShooterLowerParamsNT.idleRPS.getValue()));
    }

    private boolean upperAtTarget(Supplier<AngularVelocity> upperSpeedSupplier) {
        return shooterUpper
                .getVelocity()
                .isNear(
                        upperSpeedSupplier.get(),
                        RotationsPerSecond.of(
                                ShooterUpperParamsNT.velocityAtGoalToleranceRPS.getValue()));
    }

    private Command waitForUpperAtSpeed(Supplier<AngularVelocity> upperSpeedSupplier) {
        return Commands.waitUntil(() -> upperAtTarget(upperSpeedSupplier));
    }

    /** Completes as soon as the upper shooter reaches the current shot speed. */
    public Command waitForFeedStart() {
        return waitForUpperAtSpeed(() -> currentSolution().shooterSpeed());
    }

    public Command waitForFeedStartAtFixedDistance(DoubleSupplier distanceMeters) {
        return waitForUpperAtSpeed(
                () -> calculator.solve(distanceMeters.getAsDouble()).shooterSpeed());
    }

    /**
     * Run both drums: upper at the given speed; lower holds idle until the upper reaches speed
     * (plus the feed delay), then spins to the scaled shot speed to feed.
     */
    private Command runShooterAt(Supplier<AngularVelocity> upperSpeedSupplier) {
        return Commands.parallel(
                shooterUpper.runVelVolt(upperSpeedSupplier),
                runLowerIdle()
                        .until(() -> upperAtTarget(upperSpeedSupplier))
                        .andThen(
                                Commands.deadline(
                                        Commands.waitSeconds(FEED_DELAY_AFTER_UPPER_READY_SECONDS),
                                        runLowerIdle()))
                        .andThen(
                                shooterLower.runVelVolt(
                                        () -> lowerSpeedFor(upperSpeedSupplier.get()))));
    }

    /** Track the hood to the solution angle, or to the max angle when {@code forceMaxHood}. */
    private Command trackHood(
            Supplier<ShotSolution> solutionSupplier, BooleanSupplier forceMaxHood) {
        return hood.runMotionMagic(
                () ->
                        forceMaxHood.getAsBoolean()
                                ? ShooterConfig.HOOD_MAX_ANGLE
                                : clampHoodAngle(solutionSupplier.get().hoodAngle()));
    }

    /** Start the hopper only after the upper wheel recovers to speed under lower-shooter load. */
    private Command feedAfterUpperReadyDelay(Supplier<AngularVelocity> upperSpeedSupplier) {
        return Commands.sequence(
                Commands.deadline(waitForLoadedUpperReady(upperSpeedSupplier), hopper.idle()),
                hopper.shoot());
    }

    /** Wait until the upper wheel has recovered to target after the lower shooter starts. */
    private Command waitForLoadedUpperReady(Supplier<AngularVelocity> upperSpeedSupplier) {
        return waitForUpperAtSpeed(upperSpeedSupplier)
                .andThen(Commands.waitSeconds(FEED_DELAY_AFTER_UPPER_READY_SECONDS))
                // The lower shooter starts after the same delay in runShooterAt(). Wait one full
                // loop so the velocity input reflects that added load before checking again.
                .andThen(Commands.waitSeconds(RobotConstants.LOOPER_DT))
                .andThen(waitForUpperAtSpeed(upperSpeedSupplier));
    }

    // ---------- composed shot commands ----------

    /**
     * Spin the flywheel to the solution speed and drive the hood to the solution angle — both
     * tracking distance continuously. The lower shooter and feed start only after the upper shooter
     * reaches its target speed and the feed delay has elapsed.
     *
     * <p>Requires shooter/hood/hopper, NOT swerve; run it in parallel with an {@link
     * AutoAimCommand} which owns chassis yaw.
     */
    public Command aimAndShoot() {
        return aimAndShoot(() -> false);
    }

    public Command aimAndShoot(BooleanSupplier forceMaxHood) {
        return shootTrackingSolution(this::currentSolution, forceMaxHood);
    }

    /**
     * Shoot with the interpolation-table solution for one fixed distance, without chassis aiming.
     */
    public Command shootAtFixedDistance(DoubleSupplier distanceMeters) {
        return shootAtFixedDistance(distanceMeters, () -> false);
    }

    public Command shootAtFixedDistance(
            DoubleSupplier distanceMeters, BooleanSupplier forceMaxHood) {
        return shootTrackingSolution(
                () -> calculator.solve(distanceMeters.getAsDouble()), forceMaxHood);
    }

    /**
     * Shared body of {@link #aimAndShoot} and {@link #shootAtFixedDistance}. The solution supplier
     * decides whether the shot tracks live distance ({@link #currentSolution}, manual-override
     * aware) or one fixed distance.
     */
    private Command shootTrackingSolution(
            Supplier<ShotSolution> solutionSupplier, BooleanSupplier forceMaxHood) {
        Supplier<AngularVelocity> upperSpeedSupplier = () -> solutionSupplier.get().shooterSpeed();
        return Commands.parallel(
                runShooterAt(upperSpeedSupplier),
                trackHood(solutionSupplier, forceMaxHood),
                feedAfterUpperReadyDelay(upperSpeedSupplier));
    }

    /**
     * Timing-only shot window for autonomous use. This owns no mechanisms: compose it as the
     * deadline of the normal shoot command so autonomous and teleop use identical motor control.
     */
    public Command shotWindowWhenReadyForSeconds(double readyTimeoutSeconds, double feedSeconds) {
        Supplier<AngularVelocity> upperSpeedSupplier = () -> currentSolution().shooterSpeed();
        boolean[] loadedUpperReady = {false};

        // Bound only the wait for loaded readiness. Once the lower shooter has started and the
        // upper wheel has recovered, give the hopper the full requested feed duration.
        Command readyWindow =
                waitForLoadedUpperReady(upperSpeedSupplier)
                        .andThen(Commands.runOnce(() -> loadedUpperReady[0] = true))
                        .withTimeout(readyTimeoutSeconds);
        // The shot window ends only after a full feedSeconds measured from actual hopper start.
        // If loaded readiness times out, feeding is skipped and the auto continues.
        return Commands.runOnce(() -> loadedUpperReady[0] = false)
                .andThen(readyWindow)
                .andThen(
                        Commands.either(
                                Commands.waitSeconds(feedSeconds),
                                Commands.none(),
                                () -> loadedUpperReady[0]));
    }

    /**
     * Spin the flywheel to the (distance-tracking) solution speed and pre-position the hood,
     * WITHOUT feeding — run this in parallel with the drive into the shot pose so the flywheel is
     * already at speed on arrival. That removes the spin-up wait (up to {@code
     * readyTimeoutSeconds}) from the shot window, which is what lets the whole auto shot fit in ~2
     * s.
     *
     * <p>Deliberately does NOT run the lower shooter (feed) or hopper — those would push balls into
     * the already-spinning flywheel and shoot them out mid-drive. Only the upper drum + hood spin
     * up here; the actual feed still waits for the shot command.
     */
    public Command spinUpForShot() {
        return Commands.parallel(
                shooterUpper.runVelVolt(() -> currentSolution().shooterSpeed()),
                trackHood(this::currentSolution, () -> false));
    }

    public Command stopDrum() {
        return shooterUpper.runVelVolt(
                () -> RotationsPerSecond.of(ShooterUpperParamsNT.stopRPS.getValue()));
    }

    public Command zeroCommand() {
        return hood.zeroCommand().withTimeout(ShooterConfig.HOOD_ZEROING_TIMEOUT_SECONDS);
    }

    private Command runShooterPrespin() {
        return Commands.parallel(runUpperIdle(), runLowerIdle());
    }

    /**
     * Park the shot mechanisms: idle the flywheels, flatten the hood, and STOP THE HOPPER. The
     * hopper is commanded to idle (0 RPS) directly here so it actually stops when the shot ends,
     * instead of being left to its default command (which keeps it spinning off the intake mode).
     * Bind to {@code onFalse} of the aim trigger.
     */
    public Command idle() {
        return Commands.parallel(
                runShooterPrespin(),
                hood.runMotionMagic(ShooterConfig.HOOD_STOW_ANGLE),
                hopper.idle());
    }

    // ---------- periodic ----------

    @Override
    public void periodic() {
        Pose2d pose = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
        cachedDistanceMeters = AutoAimCommand.getDistanceToTarget(pose.getTranslation());
        cachedSolution = solutionForDistance(cachedDistanceMeters);
        cachedAimHeading = AutoAimCommand.getShooterAimHeading(pose);
        updateAimRate();
        cachedHeadingAtGoal =
                Math.abs(pose.getRotation().minus(cachedAimHeading).getDegrees())
                        <= calculator.headingToleranceDeg();

        Logger.recordOutput("Shooting/distanceMeters", cachedDistanceMeters);
        Logger.recordOutput("Shooting/manualOverride", manualOverrideEnabled());
        Logger.recordOutput("Shooting/hoodTargetDeg", cachedSolution.hoodAngle().in(Degrees));
        Logger.recordOutput(
                "Shooting/shooterUpperTargetRPS",
                cachedSolution.shooterSpeed().in(RotationsPerSecond));
        Logger.recordOutput(
                "Shooting/shooterLowerTargetRPS",
                lowerSpeedFor(cachedSolution.shooterSpeed()).in(RotationsPerSecond));
        Logger.recordOutput("Shooting/shooterAtGoal", shooterAtGoal());
        Logger.recordOutput("Shooting/headingAtGoal", cachedHeadingAtGoal);
        Logger.recordOutput("Shooting/readyToShoot", readyToShoot());

        // Visualization is debug-only telemetry. Skip it when the flywheel is idle so it stays off
        // the loop budget; the fields hold their last value in AdvantageScope while not shooting.
        if (isShooterActive()) {
            Translation2d hub = AutoAimCommand.getTarget();
            Logger.recordOutput("Shooting/Viz/Hub", new Pose2d(hub, new Rotation2d()));
            Logger.recordOutput(
                    "Shooting/Viz/AimPose", new Pose2d(pose.getTranslation(), cachedAimHeading));
        }
    }

    private void updateAimRate() {
        double timestampSec = Timer.getFPGATimestamp();
        double dtSec = timestampSec - lastAimTimestampSec;
        double raw =
                lastAimHeading == null || dtSec <= 0.0 || dtSec > 0.25
                        ? 0.0
                        : cachedAimHeading.minus(lastAimHeading).getRadians() / dtSec;
        double filtered = aimRateFilter.calculate(raw);
        // Floor sub-threshold residual to 0 so a stationary aim commands exactly zero feedforward.
        aimRate = Math.abs(filtered) < AIM_RATE_DEADBAND_RAD_S ? 0.0 : filtered;
        lastAimHeading = cachedAimHeading;
        lastAimTimestampSec = timestampSec;
    }
}
