package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

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
    // target + the NT exit velocity, rebuilt whenever params change. Autopilot adds this scalar
    // along the direction of travel, so with an entry angle set it becomes vx=exitVel, vy=0 in the
    // target frame (the robot arrives still moving, e.g. to hand off to the next path).
    private APTarget activeTarget;

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

        // Constraints placeholder; applyParams() below fills in the real (NT-tunable) values
        // before this is ever used.
        headingController =
                new ProfiledPIDController(
                        0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        // Build the Autopilot profile + heading gains/profile from the current NT params.
        applyParams();

        addRequirements(swerve);
    }

    /**
     * (Re)build the {@link Autopilot} from the current {@link AutoPilotParams} and push the heading
     * gains/profile/tolerance into the controller. Called in the constructor and from {@link
     * #execute()} whenever any param changes, so the profile and gains tune live. Autopilot itself
     * is stateless, so swapping the instance mid-run is safe; the heading controller's setters only
     * affect the next {@code calculate()}, not the profile state. The translational constraints
     * still pull from the live swerve limit each rebuild.
     *
     * <p>The heading profile's max velocity/acceleration are deliberately <b>not</b> the full
     * swerve angular limits — those are fast enough (e.g. 1000 deg/s, 5000 deg/s^2) that any turn
     * finishes in a fraction of a second, long before the translational move (which ramps up over
     * the whole remaining distance) becomes visually significant. That makes a move that's actually
     * commanded concurrently every loop look like "rotate, then translate." Tune {@code
     * headingMaxVelocityDegps}/{@code headingMaxAccelDegps2} down so the turn takes about as long
     * as the drive, and the two blend together instead.
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

        // Exit velocity: scalar magnitude Autopilot adds along the direction of travel. With the
        // entry angle set (see AutoActions.driveToPoseAutoPilot) that direction is the entry-angle
        // line, so this is effectively vx in the target frame with vy = 0.
        this.activeTarget = target.withVelocity(AutoPilotParamsNT.exitVelocityMps.getValue());

        // Internal D stays 0 — damping is applied explicitly in execute() against the measured gyro
        // rate (headingKD is that damping gain, not the PIDController's derivative term).
        headingController.setPID(
                AutoPilotParamsNT.headingKP.getValue(),
                AutoPilotParamsNT.headingKI.getValue(),
                0.0);
        headingController.setTolerance(
                Math.toRadians(AutoPilotParamsNT.errorThetaDegrees.getValue()));
        headingController.setConstraints(
                new TrapezoidProfile.Constraints(
                        Math.toRadians(AutoPilotParamsNT.headingMaxVelocityDegps.getValue()),
                        Math.toRadians(AutoPilotParamsNT.headingMaxAccelDegps2.getValue())));
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

        APResult result = autopilot.calculate(currentPose, robotRelativeSpeeds, activeTarget);

        // Translational velocity comes back field-relative.
        double vx = result.vx().in(MetersPerSecond);
        double vy = result.vy().in(MetersPerSecond);

        // Autopilot only hands us a target heading — profile our way to it for omega.
        //
        // omega = P*(profilePos - heading)          // ProfiledPIDController, P only (I/D = 0)
        //       + profileVel                        // velocity feedforward (drive at profile
        // speed)
        //       + kDamp*(profileVel - measuredRate) // damp actual rate toward the profile's rate
        //
        // The damping term is the fix for overshoot with no kI: a P(+FF) loop on an inertial axis
        // overshoots at any kP because nothing brakes based on how fast the chassis is ACTUALLY
        // turning. Damping toward the profile velocity brakes it — at the end profileVel -> 0, so
        // it becomes pure -kDamp*measuredRate braking. We damp against the measured gyro rate
        // directly (not the PIDController's own D term) so a scheduler overrun can't amplify it —
        // same reasoning as AutoAimCommand.
        Rotation2d targetAngle = result.targetAngle();
        double pTerm =
                headingController.calculate(
                        currentPose.getRotation().getRadians(),
                        new TrapezoidProfile.State(targetAngle.getRadians(), 0.0));
        double profileVel = headingController.getSetpoint().velocity;
        double measuredOmega = swerve.getYawVelocityRadPerSec();
        double omega =
                pTerm
                        + profileVel
                        + AutoPilotParamsNT.headingKD.getValue() * (profileVel - measuredOmega);

        // Field-relative (vx, vy, omega) -> robot-relative twist for runTwist.
        ChassisSpeeds speeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, currentPose.getRotation());
        swerve.runTwist(speeds);

        Logger.recordOutput("AutoPilot/TargetPose", activeTarget.getReference());
        Logger.recordOutput("AutoPilot/AtTarget", autopilot.atTarget(currentPose, activeTarget));
        Logger.recordOutput(
                "AutoPilot/DistanceToTarget",
                currentPose.getTranslation().getDistance(target.getReference().getTranslation()));
        Logger.recordOutput("AutoPilot/vxCmd", vx);
        Logger.recordOutput("AutoPilot/vyCmd", vy);
        Logger.recordOutput("AutoPilot/omegaCmd", omega);
        Logger.recordOutput("AutoPilot/targetHeadingDeg", targetAngle.getDegrees());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        // Done once Autopilot reports the pose within the profile's XY + theta tolerances. Note:
        // atTarget only checks position/heading, so a nonzero exit velocity means we finish while
        // still moving (a drive-through), handing the motion off to whatever runs next.
        return autopilot.atTarget(
                RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d(), activeTarget);
    }

    @NTParameter(tableName = "Params/AutoPilot")
    public static final class AutoPilotParams {
        // Heading-control gains for the rotational axis. Autopilot owns translation; this only
        // steers yaw toward the target angle.
        //
        // Tuning for fast response with NO overshoot (overshoot here comes from missing damping,
        // not from kI):
        //   1. Start headingKD = 0, headingKI = 0. Raise headingKP until it turns crisply and just
        //      begins to overshoot / ring at the end.
        //   2. Raise headingKD (measured-gyro-rate damping, applied in execute()) until the
        //      overshoot is gone. kD REMOVES overshoot; more kP without kD only makes it worse.
        //   3. Leave headingKI = 0. With the profile + velocity feedforward there's no steady-state
        //      error for I to fix; it would only wind up and re-introduce overshoot.
        //   4. If it's not fast enough, raise the profile limits below (they cap how fast the turn
        //      may go); kP/kD only track that profile.
        public static final double headingKP = 1.7;
        public static final double headingKI = 0.0;
        // Damping gain on measured yaw rate (applied explicitly in execute(), NOT the PID's D
        // term).
        public static final double headingKD = 0.0;

        // Exit (drive-through) speed in m/s along the entry-angle direction; 0 = stop at the pose.
        // Autopilot adds this along the direction of travel, so with an entry angle set it is vx in
        // the target frame with vy = 0. The command finishes (atTarget) while still moving at this
        // speed — only use a nonzero value when something runs right after to take over the motion.
        public static final double exitVelocityMps = 2;

        // Heading trapezoid-profile limits — deliberately independent of the full swerve chassis
        // angular limits (which are fast enough to make any turn finish near-instantly). Slow these
        // down until the turn takes roughly as long as the translational move, so the two blend
        // instead of looking like "rotate, then translate."
        public static final double headingMaxVelocityDegps = 110;
        public static final double headingMaxAccelDegps2 = 700;

        // Profile tolerances / end behavior for the translational path (see APProfile).
        public static final double errorXYMeters = 0.08;
        public static final double errorThetaDegrees = 5;
        // Under this distance Autopilot drives straight at the target and stops respecting entry
        // angle, so a small overshoot doesn't send it arcing all the way back around.
        public static final double beelineRadiusMeters = 0.5;

        // End-of-path deceleration aggressiveness (m/s^3). Higher = later, harder braking.
        public static final double jerk = 3.0;

        // Field-relative direction of travel Autopilot targets on arrival (independent of the
        // robot's final heading, which is the target pose's own rotation). Autopilot has no
        // explicit "curve left/right" knob — the swirl path bulges toward whichever side of this
        // entry-angle line the robot's current position falls on (bearing-to-target minus this
        // angle). If the default straight-line/curve sweeps through an obstacle, rotate this value
        // so the robot approaches on the other side instead.
        public static final double entryAngleDegrees = 180;
    }
}
