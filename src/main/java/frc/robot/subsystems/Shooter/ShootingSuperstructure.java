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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.RobotStateRecorder;
import frc.robot.commands.AutoAimCommand;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import java.util.function.Supplier;
import lib.ironpulse.command.VisualizeProjectileShot;
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
    private static final double HOPPER_FEED_DELAY_SECONDS = 0.1;

    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooterUpper;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooterLower;
    private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood;
    private final HopperSubsystem hopper;
    private final Swerve swerve;
    private final ShotCalculator calculator = new ShotCalculator();

    // TODO: tune kRpsToMuzzleMps until the visualized arc lands in the hub at a known, stationary
    // distance. Visualization only — does NOT affect aim (that comes from the ToF table).
    private static final double kRpsToMuzzleMps = 0.11;

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
    }

    private Pose2d robotPose() {
        return RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
    }

    /** Horizontal distance from the robot to the (alliance-flipped) hub, in meters. */
    public double distanceToTarget() {
        return AutoAimCommand.getDistanceToTarget(robotPose().getTranslation());
    }

    /**
     * Distance the shot must actually cover given chassis motion (shoot-on-move lookahead): the
     * ball inherits the chassis field velocity for its time of flight. Equals {@link
     * #distanceToTarget()} when stationary. Uses the commanded (setpoint) velocity for smoothness,
     * à la 6328.
     */
    public double effectiveDistanceToTarget() {
        ChassisSpeeds fieldVel =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        swerve.getChassisSpeedsCmd(), robotPose().getRotation());
        return calculator.effectiveDistance(
                robotPose().getTranslation(),
                AutoAimCommand.getTarget(),
                fieldVel.vxMetersPerSecond,
                fieldVel.vyMetersPerSecond);
    }

    /** The shot solution (hood angle + flywheel speed), shoot-on-move compensated. */
    public ShotSolution currentSolution() {
        return calculator.solve(effectiveDistanceToTarget());
    }

    private Rotation2d computeAimHeading() {
        ChassisSpeeds fv =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        swerve.getChassisSpeedsCmd(), robotPose().getRotation());
        double tof = calculator.timeOfFlightFor(distanceToTarget());
        Translation2d lookahead =
                robotPose()
                        .getTranslation()
                        .plus(
                                new Translation2d(
                                        fv.vxMetersPerSecond * tof, fv.vyMetersPerSecond * tof));
        return AutoAimCommand.getShooterAimHeading(
                new Pose2d(lookahead, robotPose().getRotation()));
    }

    private Rotation2d cachedAimHeading;
    private Rotation2d lastAimHeading;
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

    /** True when the SHOOTER is pointed at the hub within the (NT-tunable) heading tolerance. */
    public boolean headingAtGoal() {
        Pose2d pose = robotPose();
        Rotation2d aimHeading = aimHeading();
        double errorDeg = Math.abs(pose.getRotation().minus(aimHeading).getDegrees());
        return errorDeg <= ShootingParamsNT.headingToleranceDeg.getValue();
    }

    /** All three shot DOFs satisfied: chassis aimed, hood at angle, flywheel up to speed. */
    public boolean readyToShoot() {
        return headingAtGoal() && hood.positionAtGoal() && shooterAtGoal();
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
                upperSpeed.in(RotationsPerSecond)
                        * ShootingParamsNT.lowerShooterSpeedScale.getValue());
    }

    private Command runShooterAt(Supplier<AngularVelocity> upperSpeedSupplier) {
        return runShooterAt(upperSpeedSupplier, () -> lowerSpeedFor(upperSpeedSupplier.get()));
    }

    private Command runShooterAt(
            Supplier<AngularVelocity> upperSpeedSupplier,
            Supplier<AngularVelocity> lowerSpeedSupplier) {
        return Commands.parallel(
                shooterUpper.runVelVolt(upperSpeedSupplier),
                shooterLower.runVelVolt(lowerSpeedSupplier));
    }

    private Command runShooterPrespin() {
        return Commands.parallel(
                shooterUpper.runVelVolt(
                        () -> RotationsPerSecond.of(ShooterUpperParamsNT.idleRPS.getValue())),
                shooterLower.runVelVolt(
                        () -> RotationsPerSecond.of(ShooterLowerParamsNT.idleRPS.getValue())));
    }

    private Command feedAfterDelay() {
        return Commands.waitSeconds(HOPPER_FEED_DELAY_SECONDS).andThen(hopper.shoot());
    }

    /**
     * Spin the flywheel to the solution speed and drive the hood to the solution angle — both
     * tracking distance continuously — then feed shortly after the shooter starts spinning.
     *
     * <p>Requires shooter/hood/floor-roller, NOT swerve; run it in parallel with an {@link
     * AutoAimCommand} which owns chassis yaw.
     */
    public Command aimAndShoot() {
        return Commands.parallel(
                runShooterAt(() -> currentSolution().shooterSpeed()),
                hood.runMotionMagic(this::clampHoodAngleForSolution),
                feedAfterDelay());
    }

    public Command shootWhenReadyForSeconds(double readyTimeoutSeconds, double feedSeconds) {
        Command readyWindow =
                Commands.sequence(
                        Commands.waitSeconds(HOPPER_FEED_DELAY_SECONDS),
                        Commands.waitSeconds(feedSeconds));

        return Commands.deadline(
                readyWindow,
                runShooterAt(() -> currentSolution().shooterSpeed()),
                hood.runMotionMagic(this::clampHoodAngleForSolution),
                feedAfterDelay());
    }

    public Command feedShotForSeconds(double seconds) {
        return Commands.deadline(
                Commands.waitSeconds(HOPPER_FEED_DELAY_SECONDS + seconds),
                runShooterAt(() -> currentSolution().shooterSpeed()),
                hood.runMotionMagic(this::clampHoodAngleForSolution),
                feedAfterDelay());
    }

    public Command fixedShoot() {
        return Commands.parallel(
                runShooterAt(
                        () -> RotationsPerSecond.of(ShooterUpperParamsNT.shootRPS.getValue()),
                        () -> RotationsPerSecond.of(ShooterLowerParamsNT.shootRPS.getValue())),
                hood.runMotionMagic(ShooterConfig.HOOD_MAX_ANGLE),
                feedAfterDelay());
    }

    /**
     * Bench test: rotate the hood to the NT-tunable test angle ({@code Params/Hood/testAngleDeg}),
     * clamped to the hood limits. Flywheel/feed untouched. Bind {@code whileTrue} so the hood
     * returns to stow on release. Requires kP (and likely kG) tuned in {@code Params/Hood} or it
     * won't move.
     */
    public Command hoodToTestAngle() {
        return hood.runMotionMagic(
                () -> clampHoodAngle(Degrees.of(HoodParamsNT.testAngleDeg.getValue())));
    }

    /**
     * Bench test: spin ONLY the shooter drum (upper) at the NT-tunable test RPS ({@code
     * Params/ShooterDrum/testRPS}); the feed roller stays on its idle default. Bind {@code
     * whileTrue} so the drum drops back to idle on release. Requires kV/kP tuned in {@code
     * Params/ShooterDrum} or it won't reach speed.
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
        double geometric = distanceToTarget();
        double effective = effectiveDistanceToTarget();
        ShotSolution solution = calculator.solve(effective);
        Rotation2d heading = computeAimHeading();
        double raw =
                (lastAimHeading == null)
                        ? 0.0
                        : heading.minus(lastAimHeading).getRadians() / RobotConstants.LOOPER_DT;
        aimRate = aimRateFilter.calculate(raw);
        lastAimHeading = heading;
        cachedAimHeading = heading;

        Logger.recordOutput("Shooting/distanceMeters", geometric);
        Logger.recordOutput("Shooting/effectiveDistanceMeters", effective);
        Logger.recordOutput("Shooting/lookaheadDeltaMeters", effective - geometric);
        Logger.recordOutput("Shooting/hoodTargetDeg", solution.hoodAngle().in(Degrees));
        Logger.recordOutput(
                "Shooting/shooterUpperTargetRPS", solution.shooterSpeed().in(RotationsPerSecond));
        Logger.recordOutput(
                "Shooting/shooterLowerTargetRPS",
                lowerSpeedFor(solution.shooterSpeed()).in(RotationsPerSecond));
        Logger.recordOutput("Shooting/shooterAtGoal", shooterAtGoal());
        Logger.recordOutput("Shooting/headingAtGoal", headingAtGoal());
        Logger.recordOutput("Shooting/readyToShoot", readyToShoot());

        // --- Shoot-on-move visualization (drag these onto a 2D/3D Field in AdvantageScope) ---
        Pose2d pose = robotPose();
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
        // The internal dual the aim math uses: pretend the shooter is here, aim at the real hub.
        Translation2d virtualShooter = pose.getTranslation().plus(leadOffset);

        Logger.recordOutput("Shooting/Viz/Hub", new Pose2d(hub, new Rotation2d()));
        Logger.recordOutput(
                "Shooting/Viz/VirtualTarget", new Pose2d(virtualTarget, new Rotation2d()));
        Logger.recordOutput(
                "Shooting/Viz/VirtualShooter", new Pose2d(virtualShooter, new Rotation2d()));
        Logger.recordOutput("Shooting/Viz/AimPose", new Pose2d(pose.getTranslation(), heading));

        // --- Ballistic arc overlay (3D Field: Commands/VisualizeProjectileShot/pathWorld) ---
        // Body-fixed muzzle pose: shooter offset rotated into the field by the robot heading.
        Pose3d muzzle =
                new Pose3d(pose).plus(new Transform3d(RobotConstants.HOOD_PIVOT, new Rotation3d()));
        VisualizeProjectileShot.logPath(
                muzzle,
                // TODO: verify the shooter fires off the back (-X); drop the +180 if it points
                // forward.
                pose.getRotation().plus(Rotation2d.fromDegrees(180.0)),
                // TODO: verify hood angle sign/zero maps to up-positive launch pitch vs CAD.
                Rotation2d.fromDegrees(hood.getCurrPos().in(Degrees)),
                solution.shooterSpeed().in(RotationsPerSecond) * kRpsToMuzzleMps,
                new Translation2d(fv.vxMetersPerSecond, fv.vyMetersPerSecond),
                true,
                "");
    }
}
