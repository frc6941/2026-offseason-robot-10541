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
import java.util.function.DoubleSupplier;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.utils.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

/**
 * Corridor-lock assist for driving through a trench.
 *
 * <p>The trench openings are a narrow lateral slot at {@code X ≈ hubCenter}, traversed along
 * field-X. The opening is only ~1.28 m wide vs a ~0.97 m robot, so there is barely 0.15 m of
 * clearance per side — this command therefore locks BOTH the lateral (world Y) position and the
 * heading, leaving the driver only the through-trench (world X) axis.
 *
 * <p>Hold-to-engage: bind with {@code whileTrue} so it only runs while the button is held (a stray
 * press can't drag the robot sideways). On {@link #initialize()} it latches the nearest trench
 * center by Y and the nearest square heading (0 or π) so nothing flips mid-trench. Lateral is a
 * motion- profiled PID (soft snap — engaging off-center eases in rather than yanking at full
 * speed); heading is profiled the same way {@link AutoAimCommand} does it. Everything is
 * world-frame; the alliance flip on the driver's forward input matches the default drive command.
 */
public class AutoTrenchCommand extends Command {
    private final Swerve swerve;
    // Driver's through-trench input (forward/back). Same convention as driveWithJoystick's
    // xSupplier
    // (e.g. () -> -driverController.getLeftY()). The lateral stick is intentionally ignored.
    private final DoubleSupplier xSupplier;

    // Trench-center Y coordinates (world/blue frame). These are physical field structures at
    // absolute
    // Y, shared by both alliances, so we pick the nearest by world Y with no alliance flip.
    private static final double LEFT_TRENCH_Y = FieldConstants.LeftTrench.center.getY();
    private static final double RIGHT_TRENCH_Y = FieldConstants.RightTrench.center.getY();

    // Lateral (Y) hold and heading hold are profiled PIDs built from one deterministic parameter
    // set. Keeping this command off NT prevents stale dashboard values from changing its behavior
    // after a deploy or reboot.
    private final ProfiledPIDController yController;
    private final ProfiledPIDController headingController;

    // Latched at initialize() so the target can't flip while the robot is mid-trench.
    private double lockedTrenchY;
    private Rotation2d lockedHeading;

    public AutoTrenchCommand(Swerve swerve, DoubleSupplier xSupplier) {
        this.swerve = swerve;
        this.xSupplier = xSupplier;

        yController =
                new ProfiledPIDController(
                        AutoTrenchParams.lateralKP,
                        0.0,
                        AutoTrenchParams.lateralKD,
                        new TrapezoidProfile.Constraints(
                                AutoTrenchParams.lateralProfileMaxVel,
                                AutoTrenchParams.lateralProfileMaxAccel));
        yController.setTolerance(
                AutoTrenchParams.lateralToleranceMeters,
                AutoTrenchParams.lateralVelocityToleranceMetersPerSec);

        headingController =
                new ProfiledPIDController(
                        AutoTrenchParams.headingKP,
                        0.0,
                        AutoTrenchParams.headingKD,
                        new TrapezoidProfile.Constraints(
                                AutoTrenchParams.headingProfileMaxVel,
                                AutoTrenchParams.headingProfileMaxAccel));
        headingController.setTolerance(
                Math.toRadians(AutoTrenchParams.headingToleranceDeg),
                AutoTrenchParams.headingVelocityToleranceRadPerSec);
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        Pose2d pose = swerve.getEstimatedPose().toPose2d();

        // Snap to whichever trench center is nearer in Y, and to the nearer square heading (0 vs
        // π).
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
        ChassisSpeeds fieldVelocity =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        swerve.getChassisSpeeds(), pose.getRotation());
        yController.reset(pose.getY(), fieldVelocity.vyMetersPerSecond);
        headingController.reset(headingRad, swerve.getYawVelocityRadPerSec());
    }

    @Override
    public void execute() {
        Pose2d pose = swerve.getEstimatedPose().toPose2d();
        double maxSpeed = swerve.getSwerveLimit().maxLinearVelocity().in(MetersPerSecond);

        // Driver keeps only the through-trench axis. Forward on the stick is driver-station
        // relative,
        // so flip it into world X the same way the default drive command handles alliance.
        double forward = MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.1);
        double worldVx = (AllianceFlipUtil.shouldFlip() ? -forward : forward) * maxSpeed;

        // Lateral: profiled PID drives world Y to the latched trench center (feedforward at profile
        // speed), capped for a gentle snap.
        double vy =
                yController.calculate(pose.getY(), new TrapezoidProfile.State(lockedTrenchY, 0.0))
                        + yController.getSetpoint().velocity;
        vy =
                MathUtil.clamp(
                        vy,
                        -AutoTrenchParams.lateralOutputMaxVel,
                        AutoTrenchParams.lateralOutputMaxVel);
        if (yController.atGoal()
                || Math.abs(vy) < AutoTrenchParams.lateralOutputDeadbandMetersPerSec) {
            vy = 0.0;
        }

        // Heading: profiled PID holds the latched square heading.
        double omega =
                headingController.calculate(
                                pose.getRotation().getRadians(),
                                new TrapezoidProfile.State(lockedHeading.getRadians(), 0.0))
                        + headingController.getSetpoint().velocity;
        omega =
                MathUtil.clamp(
                        omega,
                        -AutoTrenchParams.headingOutputMaxVel,
                        AutoTrenchParams.headingOutputMaxVel);
        if (headingController.atGoal()
                || Math.abs(omega) < AutoTrenchParams.headingOutputDeadbandRadPerSec) {
            omega = 0.0;
        }

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

    public static final class AutoTrenchParams {
        // --- Lateral (Y) hold. Profiled so an off-center engage snaps in smoothly and arrives at
        // rest. ---
        public static final double lateralKP = 4.2;
        public static final double lateralKD = 0.0;
        // m/s — deliberately gentle for the soft snap.
        public static final double lateralProfileMaxVel = 3.5;
        public static final double lateralProfileMaxAccel = 7.5; // m/s^2
        public static final double lateralOutputMaxVel = 1.5;
        public static final double lateralToleranceMeters = 0.03;
        public static final double lateralVelocityToleranceMetersPerSec = 0.05;
        public static final double lateralOutputDeadbandMetersPerSec = 1.1;

        // --- Heading hold (latched to 0 or π), same profiled approach as AutoAimCommand. ---
        public static final double headingKP = 6.0;
        public static final double headingKD = 0.0;
        public static final double headingProfileMaxVel = 7.0; // rad/s
        public static final double headingProfileMaxAccel = 60.0; // rad/s^2
        public static final double headingOutputMaxVel = 1.3; // rad/s
        public static final double headingToleranceDeg = 1.0;
        public static final double headingVelocityToleranceRadPerSec = 0.05;
        public static final double headingOutputDeadbandRadPerSec = 0.03;
    }
}
