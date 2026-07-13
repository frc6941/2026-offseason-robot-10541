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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
     * Distance the shot must actually cover given chassis motion (shoot-on-move lookahead): the
     * ball inherits the chassis field velocity for its time of flight. Equals {@link
     * #distanceToTarget()} when stationary. Uses the commanded (setpoint) velocity for smoothness,
     * à la 6328.
     */
    public double effectiveDistanceToTarget() {
        return effectiveDistanceToTarget(robotPose());
    }

    private double effectiveDistanceToTarget(Pose2d pose) {
        ChassisSpeeds fieldVel =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        swerve.getChassisSpeedsCmd(), pose.getRotation());
        return calculator.effectiveDistance(
                pose.getTranslation(),
                AutoAimCommand.getTarget(),
                fieldVel.vxMetersPerSecond,
                fieldVel.vyMetersPerSecond);
    }

    /** The shot solution (hood angle + flywheel speed), shoot-on-move compensated. */
    public ShotSolution currentSolution() {
        return solutionForDistance(effectiveDistanceToTarget());
    }

    private ShotSolution solutionForDistance(double effectiveDistanceMeters) {
        if (SmartDashboard.getBoolean(MANUAL_OVERRIDE_KEY, false)) {
            Angle hoodAngle =
                    clampHoodAngle(
                            Degrees.of(SmartDashboard.getNumber(MANUAL_HOOD_ANGLE_KEY, 10.0)));
            AngularVelocity flywheelSpeed =
                    RotationsPerSecond.of(
                            Math.max(0.0, SmartDashboard.getNumber(MANUAL_FLYWHEEL_RPS_KEY, 55.0)));
            return new ShotSolution(hoodAngle, flywheelSpeed);
        }
        return calculator.solve(effectiveDistanceMeters);
    }

    private Rotation2d computeAimHeading(Pose2d pose) {
        ChassisSpeeds fv =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        swerve.getChassisSpeedsCmd(), pose.getRotation());
        double tof = calculator.timeOfFlightFor(distanceToTarget(pose));
        Translation2d lookahead =
                pose.getTranslation()
                        .plus(
                                new Translation2d(
                                        fv.vxMetersPerSecond * tof, fv.vyMetersPerSecond * tof));
        return AutoAimCommand.getShooterAimHeading(new Pose2d(lookahead, pose.getRotation()));
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
        return headingAtGoal(pose) && hood.positionAtGoal() && shooterAtGoal();
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
                        > ShooterConfig.ShooterLowerParams.idleRPS + 1.0;
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
                        RotationsPerSecond.of(ShooterConfig.ShooterLowerParams.idleRPS)));
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
                shooterLower
                        .runVelVolt(RotationsPerSecond.of(ShooterConfig.ShooterLowerParams.idleRPS))
                        .until(() -> upperAtTarget(upperSpeedSupplier))
                        .andThen(shooterLower.runVelVolt(lowerSpeedSupplier)));
    }

    private boolean upperAtTarget(Supplier<AngularVelocity> upperSpeedSupplier) {
        return shooterUpper
                .getVelocity()
                .isNear(
                        upperSpeedSupplier.get(),
                        RotationsPerSecond.of(
                                ShooterUpperParamsNT.velocityAtGoalToleranceRPS.getValue()));
    }

    private Command runShooterPrespin() {
        return Commands.parallel(
                shooterUpper.runVelVolt(
                        () -> RotationsPerSecond.of(ShooterUpperParamsNT.idleRPS.getValue())),
                shooterLower.runVelVolt(
                        RotationsPerSecond.of(ShooterConfig.ShooterLowerParams.idleRPS)));
    }

    /**
     * Feeds only while the shooter is up to speed AND the chassis is aimed at the hub; drops back
     * to idle the instant either goes false (e.g. the chassis swings off target mid-feed), instead
     * of latching into shoot() after the first ready check.
     */
    private Command feedWhenUpperReady(Supplier<AngularVelocity> upperSpeedSupplier) {
        return hopper.shootWhile(() -> upperAtTarget(upperSpeedSupplier) && headingAtGoal());
    }

    /**
     * Spin the flywheel to the solution speed and drive the hood to the solution angle — both
     * tracking distance continuously. The lower shooter and feed start only after the upper shooter
     * reaches its target speed.
     *
     * <p>Requires shooter/hood/floor-roller, NOT swerve; run it in parallel with an {@link
     * AutoAimCommand} which owns chassis yaw.
     */
    public Command aimAndShoot() {
        Supplier<AngularVelocity> upperSpeedSupplier = () -> currentSolution().shooterSpeed();
        return Commands.parallel(
                runShooterAt(upperSpeedSupplier),
                hood.runMotionMagic(this::clampHoodAngleForSolution),
                feedWhenUpperReady(upperSpeedSupplier));
    }

    public Command shootWhenReadyForSeconds(double readyTimeoutSeconds, double feedSeconds) {
        Supplier<AngularVelocity> upperSpeedSupplier = () -> currentSolution().shooterSpeed();
        Command readyWindow =
                Commands.waitUntil(() -> upperAtTarget(upperSpeedSupplier))
                        .withTimeout(readyTimeoutSeconds)
                        .andThen(
                                Commands.either(
                                        Commands.waitSeconds(feedSeconds),
                                        Commands.none(),
                                        () -> upperAtTarget(upperSpeedSupplier)));

        return Commands.deadline(
                readyWindow,
                runShooterAt(upperSpeedSupplier),
                hood.runMotionMagic(this::clampHoodAngleForSolution),
                feedWhenUpperReady(upperSpeedSupplier));
    }

    public Command feedShotForSeconds(double seconds) {
        Supplier<AngularVelocity> upperSpeedSupplier = () -> currentSolution().shooterSpeed();
        return Commands.deadline(
                Commands.waitUntil(() -> upperAtTarget(upperSpeedSupplier))
                        .andThen(Commands.waitSeconds(seconds)),
                runShooterAt(upperSpeedSupplier),
                hood.runMotionMagic(this::clampHoodAngleForSolution),
                feedWhenUpperReady(upperSpeedSupplier));
    }

    public Command fixedShoot() {
        Supplier<AngularVelocity> upperSpeedSupplier =
                () -> RotationsPerSecond.of(ShooterUpperParamsNT.shootRPS.getValue());
        return Commands.parallel(
                runShooterAt(
                        upperSpeedSupplier,
                        () -> RotationsPerSecond.of(ShooterConfig.ShooterLowerParams.shootRPS)),
                hood.runMotionMagic(ShooterConfig.HOOD_MAX_ANGLE),
                feedWhenUpperReady(upperSpeedSupplier));
    }

    /**
     * Bench test: rotate the hood to the configured test angle, clamped to the hood limits.
     * Flywheel/feed untouched. Bind {@code whileTrue} so the hood returns to stow on release.
     */
    public Command hoodToTestAngle() {
        return hood.runMotionMagic(
                () -> clampHoodAngle(Degrees.of(ShooterConfig.HoodParams.testAngleDeg)));
    }

    /**
     * Bench test: spin only the shooter drum (upper) at the configured test RPS; the feed roller
     * stays on its idle default. Bind {@code whileTrue} so the drum drops back to idle on release.
     */
    public Command spinDrumAtTestRPS() {
        return shooterUpper.runVelVolt(
                () -> RotationsPerSecond.of(ShooterUpperParamsNT.testRPS.getValue()));
    }

    public void seedHoodPositionAtZero() {
        hood.setCurrPos(ShooterConfig.HOOD_MIN_ANGLE);
    }

    /** Treat the current hood position as the zero angle without moving the mechanism. */
    public Command zeroHoodHere() {
        return Commands.runOnce(this::seedHoodPositionAtZero, hood);
    }

    public Command zeroCommand() {
        return hood.zeroCommand();
    }

    public Angle getHoodAngle() {
        return hood.getCurrPos();
    }

    /**
     * Park the shot mechanisms: stop the flywheel and flatten the hood. The floor roller is left to
     * its own default command. Bind to {@code onFalse} of the aim trigger.
     */
    public Command idle() {
        return Commands.parallel(
                runShooterPrespin(), hood.runMotionMagic(ShooterConfig.HOOD_STOW_ANGLE));
    }

    @Override
    public void periodic() {
        // Read the world pose ONCE per loop and thread it through; each robotPose() call re-reads
        // the transform buffer (TreeMap lookup + quaternion inverse), and this method used to do it
        // ~6x. See the pose-taking overloads above.
        Pose2d pose = robotPose();
        double geometric = distanceToTarget(pose);
        double effective = effectiveDistanceToTarget(pose);
        ShotSolution solution = calculator.solve(effective);
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
        Logger.recordOutput("Shooting/effectiveDistanceMeters", effective);
        Logger.recordOutput("Shooting/lookaheadDeltaMeters", effective - geometric);
        Logger.recordOutput(
                "Shooting/manualOverride", SmartDashboard.getBoolean(MANUAL_OVERRIDE_KEY, false));
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
            // --- Shoot-on-move visualization (drag these onto a 2D/3D Field in AdvantageScope) ---
            Translation2d hub = AutoAimCommand.getTarget();
            ChassisSpeeds fv =
                    ChassisSpeeds.fromRobotRelativeSpeeds(
                            swerve.getChassisSpeedsCmd(), pose.getRotation());
            double tof = calculator.timeOfFlightFor(geometric);
            // How far the ball drifts downrange from inheriting chassis velocity over its flight.
            Translation2d leadOffset =
                    new Translation2d(fv.vxMetersPerSecond * tof, fv.vyMetersPerSecond * tof);
            // The point the chassis actually aims at: the hub pulled back against our motion.
            Translation2d virtualTarget = hub.minus(leadOffset);
            // The internal dual the aim math uses: pretend the shooter is here, aim at the real
            // hub.
            Translation2d virtualShooter = pose.getTranslation().plus(leadOffset);

            Logger.recordOutput("Shooting/Viz/Hub", new Pose2d(hub, new Rotation2d()));
            Logger.recordOutput(
                    "Shooting/Viz/VirtualTarget", new Pose2d(virtualTarget, new Rotation2d()));
            Logger.recordOutput(
                    "Shooting/Viz/VirtualShooter", new Pose2d(virtualShooter, new Rotation2d()));
            Logger.recordOutput("Shooting/Viz/AimPose", new Pose2d(pose.getTranslation(), heading));

            // --- Quadratic-drag arc overlay (3D Field: Shooting/Viz/DragPath) ---
            // Body-fixed muzzle pose: shooter offset rotated into the field by the robot heading.
            Pose3d muzzle =
                    new Pose3d(pose)
                            .plus(new Transform3d(RobotConstants.HOOD_PIVOT, new Rotation3d()));
        }
    }
}
