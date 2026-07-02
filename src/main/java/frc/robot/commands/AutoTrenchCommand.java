package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.utils.AllianceFlipUtil;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

/**
 * Corridor-lock assist for driving through a trench.
 *
 * <p>The trench openings are a narrow lateral slot at {@code X ≈ hubCenter}, traversed along field-X.
 * The opening is only ~1.28 m wide vs a ~0.97 m robot, so there is barely 0.15 m of clearance per
 * side — this command therefore locks BOTH the lateral (world Y) position and the heading, leaving the
 * driver only the through-trench (world X) axis.
 *
 * <p>Hold-to-engage: bind with {@code whileTrue} so it only runs while the button is held (a stray
 * press can't drag the robot sideways). On {@link #initialize()} it latches the nearest trench center
 * by Y and the nearest square heading (0 or π) so nothing flips mid-trench. Lateral is a motion-
 * profiled PID (soft snap — engaging off-center eases in rather than yanking at full speed); heading
 * is profiled the same way {@link AutoAimCommand} does it. Everything is world-frame; the alliance
 * flip on the driver's forward input matches the default drive command.
 */
public class AutoTrenchCommand extends Command {
    private final Swerve swerve;
    // Driver's through-trench input (forward/back). Same convention as driveWithJoystick's xSupplier
    // (e.g. () -> -driverController.getLeftY()). The lateral stick is intentionally ignored.
    private final DoubleSupplier xSupplier;

    // Trench-center Y coordinates (world/blue frame). These are physical field structures at absolute
    // Y, shared by both alliances, so we pick the nearest by world Y with no alliance flip.
    private static final double LEFT_TRENCH_Y = FieldConstants.LeftTrench.center.getY();
    private static final double RIGHT_TRENCH_Y = FieldConstants.RightTrench.center.getY();

    // --- Lateral (Y) hold. Profiled so an off-center engage snaps in smoothly and arrives at rest. ---
    private static final double LATERAL_KP = 4.0;
    private static final double LATERAL_KD = 0.0;
    private static final double LATERAL_MAX_VEL = 2.0; // m/s — deliberately gentle for the soft snap
    private static final double LATERAL_MAX_ACCEL = 6.0; // m/s^2
    private final ProfiledPIDController yController =
            new ProfiledPIDController(
                    LATERAL_KP, 0.0, LATERAL_KD,
                    new TrapezoidProfile.Constraints(LATERAL_MAX_VEL, LATERAL_MAX_ACCEL));

    // --- Heading hold (latched to 0 or π), same profiled approach as AutoAimCommand. ---
    private static final double HEADING_KP = 6.0;
    private static final double HEADING_KD = 0.0;
    private static final double HEADING_MAX_VEL = 7.0; // rad/s
    private static final double HEADING_MAX_ACCEL = 60.0; // rad/s^2
    private final ProfiledPIDController headingController =
            new ProfiledPIDController(
                    HEADING_KP, 0.0, HEADING_KD,
                    new TrapezoidProfile.Constraints(HEADING_MAX_VEL, HEADING_MAX_ACCEL));

    // Latched at initialize() so the target can't flip while the robot is mid-trench.
    private double lockedTrenchY;
    private Rotation2d lockedHeading;

    public AutoTrenchCommand(Swerve swerve, DoubleSupplier xSupplier) {
        this.swerve = swerve;
        this.xSupplier = xSupplier;
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        Pose2d pose = swerve.getEstimatedPose().toPose2d();

        // Snap to whichever trench center is nearer in Y, and to the nearer square heading (0 vs π).
        lockedTrenchY =
                Math.abs(pose.getY() - LEFT_TRENCH_Y) <= Math.abs(pose.getY() - RIGHT_TRENCH_Y)
                        ? LEFT_TRENCH_Y
                        : RIGHT_TRENCH_Y;
        double headingRad = pose.getRotation().getRadians();
        lockedHeading =
                Math.abs(MathUtil.angleModulus(headingRad)) <= Math.PI / 2.0
                        ? Rotation2d.kZero
                        : Rotation2d.kPi;

        // Seed both profiles with the current state (field-frame lateral velocity + yaw rate) so
        // engaging mid-motion doesn't command a velocity discontinuity.
        ChassisSpeeds fieldVel =
                ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getChassisSpeeds(), pose.getRotation());
        yController.reset(pose.getY(), fieldVel.vyMetersPerSecond);
        headingController.reset(headingRad, swerve.getYawVelocityRadPerSec());
    }

    @Override
    public void execute() {
        Pose2d pose = swerve.getEstimatedPose().toPose2d();
        double maxSpeed = swerve.getSwerveLimit().maxLinearVelocity().in(MetersPerSecond);

        // Driver keeps only the through-trench axis. Forward on the stick is driver-station relative,
        // so flip it into world X the same way the default drive command handles alliance.
        double forward = MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.1);
        double worldVx = (AllianceFlipUtil.shouldFlip() ? -forward : forward) * maxSpeed;

        // Lateral: profiled PID drives world Y to the latched trench center (feedforward at profile
        // speed), capped for a gentle snap.
        double vy = yController.calculate(pose.getY(), new TrapezoidProfile.State(lockedTrenchY, 0.0))
                + yController.getSetpoint().velocity;
        vy = MathUtil.clamp(vy, -LATERAL_MAX_VEL, LATERAL_MAX_VEL);

        // Heading: profiled PID holds the latched square heading.
        double omega =
                headingController.calculate(
                                pose.getRotation().getRadians(),
                                new TrapezoidProfile.State(lockedHeading.getRadians(), 0.0))
                        + headingController.getSetpoint().velocity;
        omega = MathUtil.clamp(omega, -HEADING_MAX_VEL, HEADING_MAX_VEL);

        // worldVx / vy / omega are all world-frame; convert to a robot-relative twist.
        ChassisSpeeds speeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(worldVx, vy, omega, pose.getRotation());
        swerve.runTwist(speeds);

        Logger.recordOutput("AutoTrench/lockedTrenchY", lockedTrenchY);
        Logger.recordOutput("AutoTrench/lockedHeadingDeg", lockedHeading.getDegrees());
        Logger.recordOutput("AutoTrench/yErrorMeters", lockedTrenchY - pose.getY());
        Logger.recordOutput(
                "AutoTrench/headingErrorDeg", lockedHeading.minus(pose.getRotation()).getDegrees());
        Logger.recordOutput("AutoTrench/worldVxCmd", worldVx);
        Logger.recordOutput("AutoTrench/vyCmd", vy);
        Logger.recordOutput("AutoTrench/omegaCmd", omega);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.runStop();
    }

    @Override
    public boolean isFinished() {
        // Hold-to-run: the whileTrue binding ends it on button release.
        return false;
    }
}
