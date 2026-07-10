package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStateRecorder;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveLimit;
import org.littletonrobotics.junction.Logger;

/**
 * Drives the swerve to an {@link APTarget} using the Autopilot vendor library.
 *
 * <p>Autopilot is a stateless point-to-point driver: each loop we hand it the current pose, the
 * current <b>robot-relative</b> chassis speeds, and the target, and it returns the next
 * field-relative translational velocity plus a target heading. Autopilot does <i>not</i> produce an
 * angular velocity, so — mirroring {@link AutoAimCommand} — we run its target heading through a
 * profiled PID controller to get omega, then feed the combined field-relative speeds back as a
 * robot-relative twist ({@link Swerve#runTwist}).
 *
 * <p>Autopilot is obstacle-unaware; it drives a straight (or entry-angle "swirly") path, so only
 * use it where the lane is known clear.
 */
public class AutoPilotCommand extends Command {
    private final Swerve swerve;
    private final APTarget target;
    private final Autopilot autopilot;

    // Heading is motion-profiled the same way AutoAimCommand does it: the trapezoid plans the
    // deceleration so the chassis arrives at the target heading at zero angular velocity.
    private final ProfiledPIDController headingController;

    /**
     * @param swerve the drivetrain to command
     * @param target where to drive to (pose + optional entry angle / end velocity / rotation
     *     radius)
     */
    public AutoPilotCommand(Swerve swerve, APTarget target) {
        this.swerve = swerve;
        this.target = target;

        // Pull translational limits from the live swerve limit so the path respects the drivetrain.
        SwerveLimit limit = swerve.getSwerveLimit();
        APConstraints constraints =
                new APConstraints(
                        limit.maxLinearVelocity().in(MetersPerSecond),
                        limit.maxSkidAcceleration().in(MetersPerSecondPerSecond),
                        AutoPilotParams.jerk);
        APProfile profile =
                new APProfile(constraints)
                        .withErrorXY(Meters.of(AutoPilotParams.errorXYMeters))
                        .withErrorTheta(Degrees.of(AutoPilotParams.errorThetaDegrees))
                        .withBeelineRadius(Meters.of(AutoPilotParams.beelineRadiusMeters));
        this.autopilot = new Autopilot(profile);

        // Rotational constraints come from the swerve angular limits.
        headingController =
                new ProfiledPIDController(
                        AutoPilotParams.headingKP,
                        AutoPilotParams.headingKI,
                        AutoPilotParams.headingKD,
                        new TrapezoidProfile.Constraints(
                                limit.maxAngularVelocity().in(RadiansPerSecond),
                                limit.maxAngularAcceleration().in(RadiansPerSecondPerSecond)));
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(Math.toRadians(AutoPilotParams.errorThetaDegrees));

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // Seed the heading profile with the current heading + yaw rate so engaging mid-motion
        // doesn't
        // command a velocity discontinuity.
        Pose2d pose = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
        headingController.reset(
                pose.getRotation().getRadians(),
                RobotStateRecorder.getOmegaRobotCurrent().in(RadiansPerSecond));
    }

    @Override
    public void execute() {
        Pose2d currentPose = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
        // Autopilot expects the robot-relative chassis speeds (it rotates them into field frame
        // using
        // the current pose internally).
        ChassisSpeeds robotRelativeSpeeds = RobotStateRecorder.getChassisSpeeds();

        APResult result = autopilot.calculate(currentPose, robotRelativeSpeeds, target);

        // Translational velocity comes back field-relative.
        double vx = result.vx().in(MetersPerSecond);
        double vy = result.vy().in(MetersPerSecond);

        // Autopilot only hands us a target heading — profile our way to it for omega.
        Rotation2d targetAngle = result.targetAngle();
        double omega =
                headingController.calculate(
                        currentPose.getRotation().getRadians(),
                        new TrapezoidProfile.State(targetAngle.getRadians(), 0.0));
        omega += headingController.getSetpoint().velocity; // feedforward at profile speed

        // Field-relative (vx, vy, omega) -> robot-relative twist for runTwist.
        ChassisSpeeds speeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, currentPose.getRotation());
        swerve.runTwist(speeds);

        Logger.recordOutput("AutoPilot/TargetPose", target.getReference());
        Logger.recordOutput("AutoPilot/AtTarget", autopilot.atTarget(currentPose, target));
        Logger.recordOutput(
                "AutoPilot/DistanceToTarget",
                currentPose.getTranslation().getDistance(target.getReference().getTranslation()));
        Logger.recordOutput("AutoPilot/vxCmd", vx);
        Logger.recordOutput("AutoPilot/vyCmd", vy);
        Logger.recordOutput("AutoPilot/omegaCmd", omega);
        Logger.recordOutput("AutoPilot/targetHeadingDeg", targetAngle.getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.runStop();
    }

    @Override
    public boolean isFinished() {
        // Done once Autopilot reports the pose within the profile's XY + theta tolerances.
        return autopilot.atTarget(RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d(), target);
    }

    public static final class AutoPilotParams {
        // Heading-control gains for the rotational axis. Autopilot owns translation; this only
        // steers
        // yaw toward the target angle. Tune on the real robot.
        public static final double headingKP = 5.0;
        public static final double headingKI = 0.0;
        public static final double headingKD = 0.0;

        // Profile tolerances / end behavior for the translational path (see APProfile).
        public static final double errorXYMeters = 0.03;
        public static final double errorThetaDegrees = 2.0;
        // Under this distance Autopilot drives straight at the target and stops respecting entry
        // angle,
        // so a small overshoot doesn't send it arcing all the way back around.
        public static final double beelineRadiusMeters = 0.30;
        // End-of-path deceleration aggressiveness (m/s^3). Higher = later, harder braking.
        public static final double jerk = 8.0;
    }
}
