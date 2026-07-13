package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStateRecorder;
import frc.robot.subsystems.Configs.SwerveMK5Config;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.utils.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

/**
 * Bang-bang drive to a field X coordinate at full linear velocity, with a fixed horizontal (Y) jog.
 *
 * <p>Each loop it commands a field-relative velocity of {@code ±maxLinearVelocity} along world X (the
 * sign points at the target) plus, until reached, a full-speed Y component toward a target {@link
 * #DELTA_Y_METERS} from the start Y — a small sideways translation. Heading is left uncommanded. It
 * finishes the instant the robot crosses the target X (in the direction it started driving) and then
 * stops — no tolerance band, so expect coast/overshoot past the line.
 *
 * <p>Intentionally a pure bang-bang test (full speed right up to the crossing). For a smooth,
 * decelerating approach use {@code AutoActions.driveToPose} instead. Bind with {@code whileTrue} so
 * releasing the button aborts it.
 */
public class DriveToXCommand extends Command {
    // Fixed horizontal (field-Y) translation applied alongside the X drive, relative to the start Y.
    private static final double DELTA_Y_METERS = 0.3;
    // Delay after the command starts before the Y jog begins (X drives alone until then).
    private static final double Y_DELAY_SECONDS = 0.3;

    private final Swerve swerve;
    private final double targetXMeters;
    private final Timer timer = new Timer();

    // Directions of travel latched at start. The command finishes as soon as the robot crosses the X
    // target in its direction (no tolerance band, no settling).
    private boolean movingPositiveX;
    private boolean movingPositiveY;
    // targetXMeters flipped into the world frame for the current alliance, and the world Y target
    // (start Y + delta) — both latched at start.
    private double worldTargetXMeters;
    private double targetYMeters;

    public DriveToXCommand(Swerve swerve, double targetXMeters) {
        this.swerve = swerve;
        this.targetXMeters = targetXMeters;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // targetXMeters is a blue-frame X; flip it into the world frame for the current alliance.
        worldTargetXMeters = AllianceFlipUtil.applyX(targetXMeters);
        movingPositiveX = worldTargetXMeters > currentX();
        // A fixed +0.3 m horizontal jog off the start Y (a delta, so no alliance flip).
        targetYMeters = currentY() + DELTA_Y_METERS;
        movingPositiveY = targetYMeters > currentY();
        timer.restart();
        swerve.setSwerveLimit(SwerveMK5Config.kUnlimitedLimit);
    }

    private double currentX() {
        return RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d().getX();
    }

    private double currentY() {
        return RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d().getY();
    }

    private boolean crossedX() {
        return movingPositiveX ? currentX() >= worldTargetXMeters : currentX() <= worldTargetXMeters;
    }

    private boolean crossedY() {
        return movingPositiveY ? currentY() >= targetYMeters : currentY() <= targetYMeters;
    }

    @Override
    public void execute() {
        Pose2d pose = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
        double maxSpeed = swerve.getSwerveLimit().maxLinearVelocity().in(MetersPerSecond);

        // Full speed toward each target; zero an axis once it has crossed. The Y jog only starts
        // once the start-up delay has elapsed, so X drives alone for the first second.
        double vx = crossedX() ? 0.0 : (movingPositiveX ? 1.0 : -1.0) * maxSpeed;
        boolean applyY = timer.hasElapsed(Y_DELAY_SECONDS) && !crossedY();
        double vy = applyY ? (movingPositiveY ? 1.0 : -1.0) * 0.3 : 0.0;
        swerve.runTwist(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, 0.0, pose.getRotation()));

        Logger.recordOutput("DriveToX/targetXMeters", worldTargetXMeters);
        Logger.recordOutput("DriveToX/currentXMeters", pose.getX());
        Logger.recordOutput("DriveToX/targetYMeters", targetYMeters);
        Logger.recordOutput("DriveToX/currentYMeters", pose.getY());
        Logger.recordOutput("DriveToX/vxCmd", vx);
        Logger.recordOutput("DriveToX/vyCmd", vy);
    }

    @Override
    public boolean isFinished() {
        // Exit the moment the robot crosses the target X in the direction it started driving.
        return crossedX();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setSwerveLimitDefault();
        swerve.runStop();
    }
}
