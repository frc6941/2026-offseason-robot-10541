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
import lib.ntext.NTParameter;
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
    // Rebuilt from NT params on change (Autopilot has no live setters), so not final.
    private Autopilot autopilot;

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

        // Rotational constraints come from the swerve angular limits (not NT), set once here.
        SwerveLimit limit = swerve.getSwerveLimit();
        headingController =
                new ProfiledPIDController(
                        0.0,
                        0.0,
                        0.0,
                        new TrapezoidProfile.Constraints(
                                limit.maxAngularVelocity().in(RadiansPerSecond),
                                limit.maxAngularAcceleration().in(RadiansPerSecondPerSecond)));
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        // Build the Autopilot profile + heading gains from the current NT params.
        applyParams();

        addRequirements(swerve);
    }

    /**
     * (Re)build the {@link Autopilot} from the current {@link AutoPilotParams} and push the heading
     * gains/tolerance into the controller. Called in the constructor and from {@link #execute()}
     * whenever any param changes, so the profile and gains tune live. Autopilot itself is stateless,
     * so swapping the instance mid-run is safe; the heading controller's setters only affect the next
     * {@code calculate()}, not the profile state. The translational constraints still pull from the
     * live swerve limit each rebuild.
     */
    private void applyParams() {
        SwerveLimit limit = swerve.getSwerveLimit();
        APConstraints constraints =
                new APConstraints(
                        limit.maxLinearVelocity().in(MetersPerSecond),
                        limit.maxSkidAcceleration().in(MetersPerSecondPerSecond),
                        AutoPilotParamsNT.jerk.getValue());
        APProfile profile =
                new APProfile(constraints)
                        .withErrorXY(Meters.of(AutoPilotParamsNT.errorXYMeters.getValue()))
                        .withErrorTheta(Degrees.of(AutoPilotParamsNT.errorThetaDegrees.getValue()))
                        .withBeelineRadius(
                                Meters.of(AutoPilotParamsNT.beelineRadiusMeters.getValue()));
        this.autopilot = new Autopilot(profile);

        headingController.setPID(
                AutoPilotParamsNT.headingKP.getValue(),
                AutoPilotParamsNT.headingKI.getValue(),
                AutoPilotParamsNT.headingKD.getValue());
        headingController.setTolerance(
                Math.toRadians(AutoPilotParamsNT.errorThetaDegrees.getValue()));
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
        // Live tuning: rebuild the profile + heading gains when the dashboard changes any param.
        if (AutoPilotParamsNT.isAnyChanged()) applyParams();

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

    @NTParameter(tableName = "Params/AutoPilot")
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
