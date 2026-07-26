package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
import lib.ironpulse.swerve.Swerve;
import org.littletonrobotics.junction.Logger;

/**
 * Coordinates a drum-shooter shot: flywheel speed + hood angle + feed gating, driven by distance to
 * the hub via {@link ShotCalculator}.
 *
 * <p>Unlike a turret robot, yaw is actuated by the chassis (see {@link AutoAimCommand}). This
 * superstructure deliberately does NOT own swerve — it only reads swerve pose to compute distance
 * and heading error. Compose {@link #aimAndShoot()} in parallel with an {@link AutoAimCommand} so
 * the drivetrain handles yaw while this handles hood + flywheel + feed.
 */
public class ShootingSuperstructure extends SubsystemBase {
    private static final String MANUAL_OVERRIDE_KEY = "Shooter Tuning/Manual Override";
    private static final String MANUAL_HOOD_ANGLE_KEY = "Shooter Tuning/Hood Angle Deg";
    private static final String MANUAL_FLYWHEEL_RPS_KEY = "Shooter Tuning/Flywheel RPS";
    private static final double FEED_DELAY_AFTER_UPPER_READY_SECONDS = 0.1;

    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooterUpper;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooterLower;
    private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood;
    private final HopperSubsystem hopper;
    private final Swerve swerve;
    private final ShotCalculator calculator = new ShotCalculator();

    public ShootingSuperstructure(
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooterUpper,
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooterLower,
            PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood,
            HopperSubsystem hopper,
            Swerve swerve) {
        this.shooterUpper = shooterUpper;
        this.shooterLower = shooterLower;
        this.hood = hood;
        this.hopper = hopper;
        this.swerve = swerve;
        cachedAimHeading = AutoAimCommand.getShooterAimHeading(robotPose());

        SmartDashboard.setDefaultBoolean(MANUAL_OVERRIDE_KEY, false);
        SmartDashboard.setDefaultNumber(MANUAL_HOOD_ANGLE_KEY, 10.0);
        SmartDashboard.setDefaultNumber(MANUAL_FLYWHEEL_RPS_KEY, 55.0);
        if (!RobotConstants.ENABLE_NT_PARAMS) {
            SmartDashboard.putBoolean(MANUAL_OVERRIDE_KEY, false);
        }
    }

    private Pose2d robotPose() {
        return RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
    }

    /** Horizontal distance from the robot to the (alliance-flipped) hub, in meters. */
    public double distanceToTarget() {
        return distanceToTarget(robotPose());
    }

    // Pose-taking overloads let periodic() read the robot pose ONCE and thread it through,
    // instead of each helper re-reading the transform buffer (a TreeMap lookup + quaternion
    // inverse) several times per loop. See periodic().
    private double distanceToTarget(Pose2d pose) {
        return AutoAimCommand.getDistanceToTarget(pose.getTranslation());
    }

    /**
     * The stationary shot solution (hood angle + flywheel speed) for the current geometric range.
     */
    public ShotSolution currentSolution() {
        return solutionForDistance(distanceToTarget());
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

    private Rotation2d computeAimHeading(Pose2d pose) {
        return AutoAimCommand.getShooterAimHeading(pose);
    }

    // Below this magnitude the aim-heading rate is treated as zero. A stationary robot aimed at a
    // stationary hub has a true rate of 0; anything left after the moving-average filter is just
    // differentiated pose (vision) noise. Feeding that residual to AutoAim's omega feedforward
    // makes
    // the chassis dither and scrubs the modules (reads as translational jitter), so we floor it
    // here.
    private static final double AIM_RATE_DEADBAND_RAD_S = 0.05;

    private Rotation2d cachedAimHeading;
    private Rotation2d lastAimHeading;
    private double lastAimTimestampSec = Double.NaN;
    private double aimRate;
    private LinearFilter aimRateFilter =
            LinearFilter.movingAverage((int) (0.1 / RobotConstants.LOOPER_DT));

    public Rotation2d aimHeading() {
        return cachedAimHeading;
    }

    public double aimHeadingRateRadPerSec() {
        return aimRate;
    }

    private Angle clampHoodAngle(Angle angle) {
        return Degrees.of(
                MathUtil.clamp(
                        angle.in(Degrees),
                        ShooterConfig.HOOD_MIN_ANGLE.in(Degrees),
                        ShooterConfig.HOOD_MAX_ANGLE.in(Degrees)));
    }

    /** True when the shooter is pointed at the hub within the configured heading tolerance. */
    public boolean headingAtGoal() {
        return headingAtGoal(robotPose());
    }

    private boolean headingAtGoal(Pose2d pose) {
        Rotation2d aimHeading = aimHeading();
        double errorDeg = Math.abs(pose.getRotation().minus(aimHeading).getDegrees());
        return errorDeg <= calculator.headingToleranceDeg();
    }

    /** All three shot DOFs satisfied: chassis aimed, hood at angle, flywheel up to speed. */
    public boolean readyToShoot() {
        return readyToShoot(robotPose());
    }

    private boolean readyToShoot(Pose2d pose) {
        return headingAtGoal(pose) && hoodAtGoal() && shooterAtGoal();
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

    public double getUpperSetpointRPS() {
        return shooterUpper.getCurrSetpoint().in(RotationsPerSecond);
    }

    public double getLowerSetpointRPS() {
        return shooterLower.getCurrSetpoint().in(RotationsPerSecond);
    }

    public void configureDefaultCommands() {
        shooterUpper.setDefaultCommand(
                shooterUpper.runVelVolt(
                        () -> RotationsPerSecond.of(ShooterUpperParamsNT.idleRPS.getValue())));
        shooterLower.setDefaultCommand(
                shooterLower.runVelVolt(
                        () -> RotationsPerSecond.of(ShooterLowerParamsNT.idleRPS.getValue())));
        hood.setDefaultCommand(hood.runMotionMagic(ShooterConfig.HOOD_STOW_ANGLE));
    }

    private Angle clampHoodAngleForSolution() {
        return clampHoodAngle(currentSolution().hoodAngle());
    }

    private AngularVelocity lowerSpeedFor(AngularVelocity upperSpeed) {
        return RotationsPerSecond.of(
                upperSpeed.in(RotationsPerSecond) * calculator.lowerShooterSpeedScale());
    }

    private Command runShooterAt(Supplier<AngularVelocity> upperSpeedSupplier) {
        return runShooterAt(upperSpeedSupplier, () -> lowerSpeedFor(upperSpeedSupplier.get()));
    }

    private Command runShooterAt(
            Supplier<AngularVelocity> upperSpeedSupplier,
            Supplier<AngularVelocity> lowerSpeedSupplier) {
        return Commands.parallel(
                shooterUpper.runVelVolt(upperSpeedSupplier),
                runLowerIdle()
                        .until(() -> upperAtTarget(upperSpeedSupplier))
                        .andThen(
                                Commands.deadline(
                                        Commands.waitSeconds(FEED_DELAY_AFTER_UPPER_READY_SECONDS),
                                        runLowerIdle()))
                        .andThen(shooterLower.runVelVolt(lowerSpeedSupplier)));
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

    /** True when the upper shooter has reached the current distance-based shot speed. */
    public boolean upperAtShotSpeed() {
        return upperAtTarget(() -> currentSolution().shooterSpeed());
    }

    /** Completes as soon as the upper shooter reaches the current shot speed. */
    public Command waitForFeedStart() {
        return waitForUpperAtSpeed(() -> currentSolution().shooterSpeed());
    }

    public Command waitForFeedStartAtFixedDistance(DoubleSupplier distanceMeters) {
        return waitForUpperAtSpeed(
                () -> calculator.solve(distanceMeters.getAsDouble()).shooterSpeed());
    }

    private Command waitForUpperAtSpeed(Supplier<AngularVelocity> upperSpeedSupplier) {
        return Commands.waitUntil(() -> upperAtTarget(upperSpeedSupplier));
    }

    private Command runShooterPrespin() {
        return Commands.parallel(
                shooterUpper.runVelVolt(
                        () -> RotationsPerSecond.of(ShooterUpperParamsNT.idleRPS.getValue())),
                shooterLower.runVelVolt(
                        () -> RotationsPerSecond.of(ShooterLowerParamsNT.idleRPS.getValue())));
    }

    /** Start the hopper only after the upper wheel recovers to speed under lower-shooter load. */
    private Command feedAfterUpperReadyDelay(Supplier<AngularVelocity> upperSpeedSupplier) {
        Command waitForLoadedUpperReady = waitForLoadedUpperReady(upperSpeedSupplier);
        return Commands.sequence(
                Commands.deadline(waitForLoadedUpperReady, hopper.idle()), hopper.shoot());
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

    /**
     * Spin the flywheel to the solution speed and drive the hood to the solution angle — both
     * tracking distance continuously. The lower shooter and feed start only after the upper shooter
     * reaches its target speed and the feed delay has elapsed.
     *
     * <p>Requires shooter/hood/floor-roller, NOT swerve; run it in parallel with an {@link
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
     * Shared body of {@link #aimAndShoot} and {@link #shootAtFixedDistance}: run the flywheel at
     * the solution speed, track the hood to the solution angle (or max when {@code forceMaxHood}),
     * and start feed once the upper wheel is ready. The solution supplier decides whether the shot
     * tracks live distance ({@link #currentSolution}, manual-override aware) or one fixed distance.
     */
    private Command shootTrackingSolution(
            Supplier<ShotSolution> solutionSupplier, BooleanSupplier forceMaxHood) {
        Supplier<AngularVelocity> upperSpeedSupplier = () -> solutionSupplier.get().shooterSpeed();
        return Commands.parallel(
                runShooterAt(upperSpeedSupplier),
                hood.runMotionMagic(
                        () ->
                                forceMaxHood.getAsBoolean()
                                        ? ShooterConfig.HOOD_MAX_ANGLE
                                        : clampHoodAngle(solutionSupplier.get().hoodAngle())),
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

    public Command shootWhenReadyForSeconds(double readyTimeoutSeconds, double feedSeconds) {
        return Commands.deadline(
                shotWindowWhenReadyForSeconds(readyTimeoutSeconds, feedSeconds), aimAndShoot());
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
                hood.runMotionMagic(this::clampHoodAngleForSolution));
    }

    public Command feedShotForSeconds(double seconds) {
        return Commands.deadline(
                Commands.waitUntil(this::upperAtShotSpeed)
                        .andThen(
                                Commands.waitSeconds(
                                        FEED_DELAY_AFTER_UPPER_READY_SECONDS + seconds)),
                aimAndShoot());
    }

    public Command fixedShoot() {
        Supplier<AngularVelocity> upperSpeedSupplier =
                () -> RotationsPerSecond.of(ShooterUpperParamsNT.shootRPS.getValue());
        return Commands.parallel(
                runShooterAt(
                        upperSpeedSupplier,
                        () -> RotationsPerSecond.of(ShooterLowerParamsNT.shootRPS.getValue())),
                hood.runMotionMagic(ShooterConfig.HOOD_MAX_ANGLE),
                feedAfterUpperReadyDelay(upperSpeedSupplier));
    }

    /**
     * Bench test: rotate the hood to the configured test angle, clamped to the hood limits.
     * Flywheel/feed untouched. Bind {@code whileTrue} so the hood returns to stow on release.
     */
    public Command hoodToTestAngle() {
        return hood.runMotionMagic(
                () -> clampHoodAngle(Degrees.of(HoodParamsNT.testAngleDeg.getValue())));
    }

    /**
     * Bench test: spin only the shooter drum (upper) at the configured test RPS; the feed roller
     * stays on its idle default. Bind {@code whileTrue} so the drum drops back to idle on release.
     */
    public Command spinDrumAtTestRPS() {
        return shooterUpper.runVelVolt(
                () -> RotationsPerSecond.of(ShooterUpperParamsNT.testRPS.getValue()));
    }

    public Command stopDrum() {
        return shooterUpper.runVelVolt(
                () -> RotationsPerSecond.of(ShooterUpperParamsNT.stopRPS.getValue()));
    }

    public void seedHoodPositionAtZero() {
        hood.setCurrPos(ShooterConfig.HOOD_MIN_ANGLE);
    }

    /** Treat the current hood position as the zero angle without moving the mechanism. */
    public Command zeroHoodHere() {
        return Commands.runOnce(this::seedHoodPositionAtZero, hood);
    }

    public Command zeroCommand() {
        return hood.zeroCommand().withTimeout(ShooterConfig.HOOD_ZEROING_TIMEOUT_SECONDS);
    }

    public Angle getHoodAngle() {
        return hood.getCurrPos();
    }

    /**
     * Park the shot mechanisms: stop the flywheel, flatten the hood, and STOP THE HOPPER. The
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

    @Override
    public void periodic() {
        // Read the world pose ONCE per loop and thread it through; each robotPose() call re-reads
        // the transform buffer (TreeMap lookup + quaternion inverse), and this method used to do it
        // ~6x. See the pose-taking overloads above.
        Pose2d pose = robotPose();
        double geometric = distanceToTarget(pose);
        ShotSolution solution = solutionForDistance(geometric);
        Rotation2d heading = computeAimHeading(pose);
        double timestampSec = Timer.getFPGATimestamp();
        double dtSec = timestampSec - lastAimTimestampSec;
        double raw =
                lastAimHeading == null || dtSec <= 0.0 || dtSec > 0.25
                        ? 0.0
                        : heading.minus(lastAimHeading).getRadians() / dtSec;
        double filtered = aimRateFilter.calculate(raw);
        // Floor sub-threshold residual to 0 so a stationary aim commands exactly zero feedforward.
        aimRate = Math.abs(filtered) < AIM_RATE_DEADBAND_RAD_S ? 0.0 : filtered;
        lastAimHeading = heading;
        lastAimTimestampSec = timestampSec;
        cachedAimHeading = heading;

        Logger.recordOutput("Shooting/distanceMeters", geometric);
        Logger.recordOutput("Shooting/manualOverride", manualOverrideEnabled());
        Logger.recordOutput("Shooting/hoodTargetDeg", solution.hoodAngle().in(Degrees));
        Logger.recordOutput(
                "Shooting/shooterUpperTargetRPS", solution.shooterSpeed().in(RotationsPerSecond));
        Logger.recordOutput(
                "Shooting/shooterLowerTargetRPS",
                lowerSpeedFor(solution.shooterSpeed()).in(RotationsPerSecond));
        Logger.recordOutput("Shooting/shooterAtGoal", shooterAtGoal());
        Logger.recordOutput("Shooting/headingAtGoal", headingAtGoal(pose));
        Logger.recordOutput("Shooting/readyToShoot", readyToShoot(pose));

        // Visualization is debug-only telemetry (pose math + a full projectile-arc log). Skip it
        // when the flywheel is idle so it stays off the loop budget; the fields simply hold their
        // last value in AdvantageScope while not shooting.
        if (isShooterActive()) {
            Translation2d hub = AutoAimCommand.getTarget();
            Logger.recordOutput("Shooting/Viz/Hub", new Pose2d(hub, new Rotation2d()));
            Logger.recordOutput("Shooting/Viz/AimPose", new Pose2d(pose.getTranslation(), heading));

            // --- Quadratic-drag arc overlay (3D Field: Shooting/Viz/DragPath) ---
            // Body-fixed muzzle pose: shooter offset rotated into the field by the robot heading.
            Pose3d muzzle =
                    new Pose3d(pose)
                            .plus(new Transform3d(RobotConstants.HOOD_PIVOT, new Rotation3d()));
        }
    }
}
