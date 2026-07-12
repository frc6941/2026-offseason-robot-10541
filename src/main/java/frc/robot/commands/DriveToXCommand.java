package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStateRecorder;
import frc.robot.subsystems.Configs.SwerveMK5Config;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.utils.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

/**
 * Bang-bang drive to a field X coordinate at full linear velocity, holding nothing else.
 *
 * <p>Each loop it commands a field-relative velocity of {@code ±maxLinearVelocity} along world X —
 * the sign points at the target — with <b>zero Y velocity and zero omega</b>, so the robot drives
 * straight along X and its Y/heading are simply left uncommanded (they shouldn't change on a clean
 * straight push). It finishes the instant the robot crosses the target X (in the direction it
 * started driving) and then stops — no tolerance band, so expect coast/overshoot past the line.
 *
 * <p>This is intentionally a pure bang-bang test: it runs full speed right up to the tolerance band,
 * so expect some coast/overshoot past the target from momentum. For a smooth, decelerating approach
 * use {@code AutoActions.driveToPose} instead. Bind with {@code whileTrue} so releasing the button
 * aborts it.
 */
public class DriveToXCommand extends Command {
    private final Swerve swerve;
    private final double targetXMeters;

    // Direction of travel latched at start: true if the target is ahead in +X. The command finishes
    // as soon as the robot crosses the target in this direction (no tolerance band, no settling).
    private boolean movingPositive;
    // targetXMeters flipped into the world frame for the current alliance (latched at start).
    private double worldTargetXMeters;

    public DriveToXCommand(Swerve swerve, double targetXMeters) {
        this.swerve = swerve;
        this.targetXMeters = targetXMeters;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // targetXMeters is a blue-frame X; flip it into the world frame for the current alliance.
        worldTargetXMeters = AllianceFlipUtil.applyX(targetXMeters);
        movingPositive = worldTargetXMeters > currentX();
        swerve.setSwerveLimit(SwerveMK5Config.kUnlimitedLimit);
    }

    private double currentX() {
        return RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d().getX();
    }

    @Override
    public void execute() {
        Pose2d pose = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
        double error = worldTargetXMeters - pose.getX();
        double maxSpeed = swerve.getSwerveLimit().maxLinearVelocity().in(MetersPerSecond);

        // Full speed in the latched direction along field +X; no Y, no rotation.
        double vx = (movingPositive ? 1.0 : -1.0) * maxSpeed;
        swerve.runTwist(ChassisSpeeds.fromFieldRelativeSpeeds(vx, 0.0, 0.0, pose.getRotation()));

        Logger.recordOutput("DriveToX/targetXMeters", worldTargetXMeters);
        Logger.recordOutput("DriveToX/currentXMeters", pose.getX());
        Logger.recordOutput("DriveToX/errorMeters", error);
        Logger.recordOutput("DriveToX/vxCmd", vx);
    }

    @Override
    public boolean isFinished() {
        // Exit the moment the robot crosses the target X in the direction it started driving.
        return movingPositive ? currentX() >= worldTargetXMeters : currentX() <= worldTargetXMeters;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setSwerveLimitDefault();
        swerve.runStop();
    }
}
