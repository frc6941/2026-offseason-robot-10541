package frc.robot.auto;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotStateRecorder;
import frc.robot.commands.AutoAimCommand;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.subsystems.Shooter.ShootingSuperstructure;
import java.io.IOException;
import java.util.Collections;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import lib.ironpulse.math.rbd.TransformRecorder;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.commands.SwerveAimToHeading;
import lib.ironpulse.swerve.commands.SwerveDriveToPose;
import lib.ironpulse.utils.AllianceFlipUtil;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

/**
 * Low-level autonomous building blocks, ported from the competition robot's {@code AutoActions}.
 *
 * <p>Driving is done the <b>PathPlanner</b> way: {@link #followPathFile} loads a {@code .path} file
 * (authored in the GUI under {@code deploy/pathplanner/paths}) and follows it with a {@link
 * FollowPathCommand}. {@link #drivePastSlope} crosses the bump and stops once stable, and {@link
 * #driveToPose} handles precise hardcoded moves.
 *
 * <p><b>Key difference from the competition robot:</b> that robot has a turret and shoots while
 * moving; this robot rotates the whole chassis to aim. So {@link #aimAtHub} spins the chassis
 * toward the hub (via {@link AutoAimCommand}) while {@link #shootAtHub} runs the shooter/feed —
 * combined in {@link #aimAndShootAtHub}.
 *
 * <p>Poses are authored in the <b>blue</b> field frame for the <b>right</b> side; the left variants
 * are the Y-mirror ({@link #mirrorY}). Wrap with {@link AllianceFlipUtil#apply} at use. These are
 * PLACEHOLDERS derived from the current path files — verify on the real field.
 */
public class AutoActions {
    // ---- Start poses (robot begins here; used by resetOnPose in sim). ----
    public static final Pose2d kTrenchStartR = new Pose2d(4.4, 0.562, Rotation2d.fromDegrees(0));
    public static final Pose2d kTrenchStartL = mirrorY(kTrenchStartR);
    // Bump start begins here (alliance side, 45 deg), then drives past the slope into neutral.
    public static final Pose2d kBumpStartR = new Pose2d(3.3, 2.56, Rotation2d.fromDegrees(45));
    public static final Pose2d kBumpStartL = mirrorY(kBumpStartR);

    // ---- Second-sweep entry pose (drive here before the second sweep path). ----
    public static final Pose2d kSecondSweepStartR = new Pose2d(3, 0.683, Rotation2d.fromDegrees(0));
    public static final Pose2d kSecondSweepStartL = mirrorY(kSecondSweepStartR);

    // ---- Slope crossing poses. Front = neutral side, End = alliance side (drive-back target).
    // ----
    public static final Pose2d kSlopeFrontR = new Pose2d(5.84, 2.56, Rotation2d.fromDegrees(-135));
    public static final Pose2d kSlopeFrontL = mirrorY(kSlopeFrontR);
    public static final Pose2d kSlopeEndR = new Pose2d(3, 2.56, Rotation2d.fromDegrees(-135));
    public static final Pose2d kSlopeEndL = mirrorY(kSlopeEndR);

    private static Swerve swerve;
    private static ShootingSuperstructure shootingSuperstructure;
    private static IntakerSubsystem intake;

    public static void init(
            Swerve swerve, ShootingSuperstructure shootingSuperstructure, IntakerSubsystem intake) {
        AutoActions.swerve = swerve;
        AutoActions.shootingSuperstructure = shootingSuperstructure;
        AutoActions.intake = intake;
    }

    /** Mirror a blue-frame pose across the field's horizontal centerline (right pose -> left). */
    private static Pose2d mirrorY(Pose2d p) {
        return new Pose2d(
                p.getX(), FieldConstants.fieldWidth - p.getY(), p.getRotation().unaryMinus());
    }

    // =====================================================================================
    // Driving — PathPlanner path following
    // =====================================================================================

    /** Follow an already-built (and already alliance-oriented) PathPlanner path. */
    public static Command followPath(PathPlannerPath path) {
        return new FollowPathCommand(
                        path,
                        () -> RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d(),
                        swerve::getChassisSpeeds,
                        (vel, ff) -> swerve.runTwist(vel),
                        new TunablePathController(),
                        RobotConstants.AUTO_ROBOT_CONFIG,
                        () -> false, // already flipped/mirrored before we get here
                        swerve)
                .beforeStarting(
                        () ->
                                Logger.recordOutput(
                                        "Auto/Traj", path.getPathPoses().toArray(new Pose2d[0])));
    }

    /**
     * A {@link PathFollowingController} that reads its PID from {@code Params/AutoPath} live. The
     * vendored {@link PPHolonomicDriveController} bakes gains into private final PID controllers with
     * no setters, so we wrap one and rebuild it whenever any AutoPath param changes ({@code
     * isAnyChanged()} is true for exactly the loop(s) after a dashboard edit). Rebuilding drops the
     * integral accumulators, which is harmless with the kI≈0 path gains and only happens the instant
     * you retune. This makes {@link #followPath} honor NT edits mid-path, not just at construction.
     */
    private static final class TunablePathController implements PathFollowingController {
        private PPHolonomicDriveController inner = build();

        private static PPHolonomicDriveController build() {
            return new PPHolonomicDriveController(
                    new PIDConstants(
                            AutoPathParamsNT.kpStrave.getValue(),
                            AutoPathParamsNT.kiStrave.getValue(),
                            AutoPathParamsNT.kdStrave.getValue()),
                    new PIDConstants(
                            AutoPathParamsNT.kpSpin.getValue(),
                            AutoPathParamsNT.kiSpin.getValue(),
                            AutoPathParamsNT.kdSpin.getValue()),
                    RobotConstants.LOOPER_DT);
        }

        @Override
        public ChassisSpeeds calculateRobotRelativeSpeeds(
                Pose2d currentPose,
                com.pathplanner.lib.trajectory.PathPlannerTrajectoryState targetState) {
            if (AutoPathParamsNT.isAnyChanged()) inner = build();
            return inner.calculateRobotRelativeSpeeds(currentPose, targetState);
        }

        @Override
        public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
            inner.reset(currentPose, currentSpeeds);
        }

        @Override
        public boolean isHolonomic() {
            return true;
        }
    }

    /**
     * Load a {@code .path} file, optionally mirror it (for the left/right symmetric variant), flip
     * it for the current alliance, then follow it. Right-authored paths pass {@code mirror =
     * isLeft}.
     */
    public static Command followPathFile(String pathName, boolean shouldMirror) {
        return Commands.defer(
                () -> {
                    PathPlannerPath path;
                    try {
                        path = PathPlannerPath.fromPathFile(pathName);
                    } catch (IOException | ParseException e) {
                        throw new RuntimeException("Failed to load path file: " + pathName, e);
                    }
                    path = shouldMirror ? path.mirrorPath() : path;
                    if (AllianceFlipUtil.shouldFlip()) {
                        path = path.flipPath();
                    }
                    return followPath(path);
                },
                Set.of(swerve));
    }

    /** Build a straight-line path on the fly through the given blue-frame waypoints. */
    public static PathPlannerPath generatePath(
            List<Pose2d> waypoints,
            List<RotationTarget> rotationTargets,
            double maxVel,
            double maxAcc,
            double endVelMps) {
        PathConstraints constraints = new PathConstraints(maxVel, maxAcc, 15.0, 40.0, 12.0);
        List<Waypoint> pts = PathPlannerPath.waypointsFromPoses(waypoints);
        Pose2d lastPose = waypoints.get(waypoints.size() - 1);
        GoalEndState endState = new GoalEndState(endVelMps, lastPose.getRotation());
        return new PathPlannerPath(
                pts,
                rotationTargets,
                Collections.emptyList(),
                Collections.emptyList(),
                Collections.emptyList(),
                constraints,
                null,
                endState,
                false);
    }

    // =====================================================================================
    // Driving — precise point moves
    // =====================================================================================

    /** Drive to an <b>already alliance-flipped</b> pose with the pose controller. */
    static Command driveToPose(Supplier<Pose2d> targetPoseSupplier) {
        return Commands.defer(
                () -> {
                    Pose2d targetPose = targetPoseSupplier.get();
                    return new SwerveDriveToPose(
                                    swerve,
                                    RobotStateRecorder::getPoseWorldRobotCurrent,
                                    () -> new Pose3d(targetPose),
                                    RobotStateRecorder::getVelocityWorldRobotCurrent,
                                    new PIDController(
                                            AutoPoseParamsNT.kpStrave.getValue(),
                                            AutoPoseParamsNT.kiStrave.getValue(),
                                            AutoPoseParamsNT.kdStrave.getValue()),
                                    new PIDController(
                                            AutoPoseParamsNT.kpSpin.getValue(),
                                            AutoPoseParamsNT.kiSpin.getValue(),
                                            AutoPoseParamsNT.kdSpin.getValue()),
                                    Meters.of(AutoPoseParamsNT.tolerancePositionM.getValue()),
                                    Degrees.of(AutoPoseParamsNT.toleranceHeadingDeg.getValue()))
                            .beforeStarting(
                                    Commands.runOnce(
                                            () ->
                                                    Logger.recordOutput(
                                                            "Auto/targetPose", targetPose)));
                },
                Set.of(swerve));
    }

    public static Command driveToPose(Pose2d targetPose) {
        return driveToPose(() -> targetPose);
    }

    static Command driveToHeading(Supplier<Rotation2d> targetRotationSupplier) {
        return Commands.defer(
                () ->
                        new SwerveAimToHeading(
                                swerve,
                                RobotStateRecorder::getPoseWorldRobotCurrent,
                                targetRotationSupplier,
                                new PIDController(
                                        AutoPoseParamsNT.kpSpin.getValue(),
                                        AutoPoseParamsNT.kiSpin.getValue(),
                                        AutoPoseParamsNT.kdSpin.getValue()),
                                Degrees.of(AutoPoseParamsNT.toleranceHeadingDeg.getValue()),
                                () -> DegreesPerSecond.of(80)),
                Set.of(swerve));
    }

    // =====================================================================================
    // Drive past the slope / bump (ported from the competition robot)
    // =====================================================================================

    /**
     * Drive across the bump toward {@code isToNeutral ? slopeFront : slopeEnd} and stop as soon as
     * the robot has crossed the bump line and its pitch has settled. Use {@code isToNeutral =
     * false} to come back to the alliance side after a sweep.
     */
    public static Command drivePastSlope(boolean isLeft, boolean isToNeutral) {
        // Default cross out to neutral holds the bump-start heading (used by the initial bump
        // start).
        return drivePastSlope(
                isLeft, isToNeutral, (isLeft ? kBumpStartL : kBumpStartR).getRotation());
    }

    /**
     * As {@link #drivePastSlope(boolean, boolean)}, but when crossing out to neutral the chassis
     * holds {@code neutralHoldHeading} (blue frame) the whole way instead of the bump-start
     * heading. Used by the bump-again second sweep, which crosses while keeping the slope-end
     * heading it already carries rather than spinning back to the bump-start rotation.
     */
    public static Command drivePastSlope(
            boolean isLeft, boolean isToNeutral, Rotation2d neutralHoldHeading) {
        if (isToNeutral) {
            // Out to neutral: drive toward the slope front and stop once over the bump line and
            // settled. The slope-front translation only sets DIRECTION; the stop is the bump
            // crossing. Hold neutralHoldHeading the whole way across (not kSlopeFront's) so the
            // chassis doesn't spin while climbing the slope.
            Pose2d slopeFront = isLeft ? kSlopeFrontL : kSlopeFrontR;
            Pose2d target = new Pose2d(slopeFront.getTranslation(), neutralHoldHeading);
            return Commands.defer(
                    () ->
                            Commands.deadline(
                                    waitCrossedBump(true),
                                    driveToPose(AllianceFlipUtil.apply(target))),
                    Set.of(swerve));
        }
        // Back: actually drive TO kSlopeEnd (so editing that pose moves where we end up). Timeout
        // so it can't stall if SwerveDriveToPose never nails the position + heading tolerance.
        return driveToPose(() -> AllianceFlipUtil.apply(isLeft ? kSlopeEndL : kSlopeEndR))
                .withTimeout(3.0);
    }

    /** Blue-frame heading the robot carries at the end of the back trip (drive to slope end). */
    public static Rotation2d slopeEndHeading(boolean isLeft) {
        return (isLeft ? kSlopeEndL : kSlopeEndR).getRotation();
    }

    static Command waitCrossedBump(boolean isToNeutral) {
        return Commands.waitUntil(() -> hasCrossedBump(isToNeutral) && isPitchStable());
    }

    public static boolean hasCrossedBump(boolean isToNeutral) {
        return isToNeutral
                ? AllianceFlipUtil.applyX(getRobotX())
                        > FieldConstants.LinesVertical.neutralZoneNear
                : AllianceFlipUtil.applyX(getRobotX()) < 3.3;
    }

    public static boolean isPitchStable() {
        return Math.abs(swerve.getPitchVelocityRadPerSec()) < 1.5
                && Math.abs(swerve.getPitchPosRad()) < 0.03;
    }

    // =====================================================================================
    // Intake + shooting (non-turret: rotate the chassis to aim, then empty the hopper)
    // =====================================================================================

    public static Command intake() {
        return intake.runIntake();
    }

    public static Command feed() {
        return intake.runFeed();
    }

    public static Command retractIntake() {
        return intake.runRetract();
    }

    /** Continuously rotate the chassis so the shooter faces the hub (no translation). */
    public static Command aimAtHub() {
        return new AutoAimCommand(
                swerve,
                () -> 0.0,
                () -> 0.0,
                shootingSuperstructure::aimHeading,
                shootingSuperstructure::aimHeadingRateRadPerSec);
    }

    /** Spin up and feed for {@code feedSeconds}; size it to empty a full hopper. */
    public static Command shootAtHub(double feedSeconds) {
        return shootingSuperstructure.shootWhenReadyForSeconds(
                AutoShootParamsNT.readyTimeoutSeconds.getValue(), feedSeconds);
    }

    /**
     * Hold the chassis aimed at the hub while emptying the hopper, then spin the drum back to idle.
     */
    public static Command aimAndShootAtHub(double feedSeconds) {
        return Commands.sequence(
                Commands.deadline(shootAtHub(feedSeconds), aimAtHub()),
                // The auto is one big sequence that holds the shooter requirement, so its default
                // idle can't run on its own — explicitly drop the drum to idle RPS after the shot.
                shootingSuperstructure.idle().withTimeout(0.02));
    }

    public static Command aimAndShootAtHub() {
        return aimAndShootAtHub(AutoShootParamsNT.feedSeconds.getValue());
    }

    /**
     * One collect/score cycle: follow {@code sweepPath} with the intake on, drive back across the
     * slope, then aim at the hub and empty the hopper.
     */
    public static Command sweepCollectShoot(String sweepPath, boolean isLeft) {
        return Commands.sequence(
                Commands.deadline(followPathFile(sweepPath, isLeft), intake()),
                drivePastSlope(isLeft, false),
                aimAndShootAtHub());
    }

    // =====================================================================================
    // Housekeeping
    // =====================================================================================

    public static Command zeroEverything() {
        return Commands.parallel(shootingSuperstructure.zeroHoodHere(), intake.zeroCommand());
    }

    /** In sim only, snap the pose estimator + transform tree to a known blue start pose. */
    public static Command resetOnPose(Pose2d bluePose) {
        Pose3d resetPose = new Pose3d(AllianceFlipUtil.apply(bluePose));
        return SwerveCommands.reset(swerve, resetPose)
                .alongWith(
                        Commands.runOnce(
                                () ->
                                        RobotStateRecorder.getInstance()
                                                .resetTransform(
                                                        TransformRecorder.kFrameWorld,
                                                        TransformRecorder.kFrameRobot)))
                .onlyIf(Robot::isSimulation)
                .ignoringDisable(true);
    }

    static double getRobotX() {
        return RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d().getX();
    }

    static double getRobotY() {
        return RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d().getY();
    }
}
