package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AutoAimCommand;
import frc.robot.subsystems.FloorRoller.FloorRollerSubsystem;
import lib.ironpulse.swerve.Swerve;
import org.littletonrobotics.junction.Logger;

/**
 * Coordinates a drum-shooter shot: flywheel speed + hood angle + feed gating, driven by distance to
 * the hub via {@link ShotCalculator}.
 *
 * <p>Unlike a turret robot, yaw is actuated by the chassis (see {@link AutoAimCommand}). This
 * superstructure deliberately does NOT own swerve — it only reads swerve pose to compute distance
 * and heading error. Compose {@link #aimAndShoot()} in parallel with an {@link AutoAimCommand} so the
 * drivetrain handles yaw while this handles hood + flywheel + feed.
 */
public class ShootingSuperstructure extends SubsystemBase {
    // TODO: TUNE — chassis heading error allowed before we consider ourselves aimed at the hub.
    private static final Rotation2d HEADING_TOLERANCE = Rotation2d.fromDegrees(2.0);

    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final FloorRollerSubsystem floorRoller;
    private final Swerve swerve;
    private final ShotCalculator calculator = new ShotCalculator();

    public ShootingSuperstructure(
            ShooterSubsystem shooter,
            HoodSubsystem hood,
            FloorRollerSubsystem floorRoller,
            Swerve swerve) {
        this.shooter = shooter;
        this.hood = hood;
        this.floorRoller = floorRoller;
        this.swerve = swerve;
    }

    private Pose2d robotPose() {
        return swerve.getEstimatedPose().toPose2d();
    }

    /** Horizontal distance from the robot to the (alliance-flipped) hub, in meters. */
    public double distanceToTarget() {
        return AutoAimCommand.getDistanceToTarget(robotPose().getTranslation());
    }

    /** The shot solution (hood angle + flywheel speed) for the current distance. */
    public ShotSolution currentSolution() {
        return calculator.solve(distanceToTarget());
    }

    /** True when the chassis is pointed at the hub within {@link #HEADING_TOLERANCE}. */
    public boolean headingAtGoal() {
        Pose2d pose = robotPose();
        Translation2d toTarget = AutoAimCommand.getTarget().minus(pose.getTranslation());
        double errorDeg = Math.abs(pose.getRotation().minus(toTarget.getAngle()).getDegrees());
        return errorDeg <= HEADING_TOLERANCE.getDegrees();
    }

    /** All three shot DOFs satisfied: chassis aimed, hood at angle, flywheel up to speed. */
    public boolean readyToShoot() {
        return headingAtGoal() && hood.positionAtGoal() && shooter.velocityAtGoal();
    }

    /**
     * Spin the flywheel to the solution speed and drive the hood to the solution angle — both
     * tracking distance continuously — then feed once {@link #readyToShoot()}.
     *
     * <p>Requires shooter/hood/floor-roller, NOT swerve; run it in parallel with an
     * {@link AutoAimCommand} which owns chassis yaw.
     */
    public Command aimAndShoot() {
        return Commands.parallel(
                shooter.runVelVolt(() -> currentSolution().shooterSpeed()),
                hood.runMotionMagic(() -> currentSolution().hoodAngle()),
                Commands.waitUntil(this::readyToShoot).andThen(floorRoller.feed()));
    }

    /**
     * Park the shot mechanisms: stop the flywheel and flatten the hood. The floor roller is left to
     * its own default command. Bind to {@code onFalse} of the aim trigger.
     */
    public Command idle() {
        return Commands.parallel(shooter.stop(), hood.setFlat());
    }

    @Override
    public void periodic() {
        double distance = distanceToTarget();
        ShotSolution solution = calculator.solve(distance);
        Logger.recordOutput("Shooting/distanceMeters", distance);
        Logger.recordOutput("Shooting/hoodTargetDeg", solution.hoodAngle().in(Degrees));
        Logger.recordOutput("Shooting/shooterTargetRPS", solution.shooterSpeed().in(RotationsPerSecond));
        Logger.recordOutput("Shooting/headingAtGoal", headingAtGoal());
        Logger.recordOutput("Shooting/readyToShoot", readyToShoot());
    }
}
