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
import com.therekrab.autopilot.APTarget;
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
import frc.robot.commands.AutoPilotCommand;
import frc.robot.commands.AutoPilotParamsNT;
import frc.robot.subsystems.Configs.SwerveMK5Config;
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
    public static final Pose2d kTrenchStartR = new Pose2d(4.4, 0.562, Rotation2d.fromDegrees(90));
    public static final Pose2d kTrenchStartL = mirrorY(kTrenchStartR);
    // Bump start begins here (alliance side, 45 deg), then drives past the slope into neutral.
    public static final Pose2d kBumpStartR = new Pose2d(3.3, 2.84, Rotation2d.fromDegrees(45));
    public static final Pose2d kBumpStartL = mirrorY(kBumpStartR);

    // ---- Second-sweep entry pose (drive here before the second sweep path). ----
    public static final Pose2d kSecondSweepStartR = new Pose2d(3, 0.562, Rotation2d.fromDegrees(0));
    public static final Pose2d kSecondSweepStartL = mirrorY(kSecondSweepStartR);

    // ---- Slope crossing poses. Front = neutral side, End = alliance side (drive-back target).
    // ----
    public static final Pose2d kSlopeFrontR = new Pose2d(5.84, 2.84, Rotation2d.fromDegrees(-135));
    public static final Pose2d kSlopeFrontL = mirrorY(kSlopeFrontR);
    public static final Pose2d kSlopeEndR = new Pose2d(3, 2.84, Rotation2d.fromDegrees(-135));
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
     * vendored {@link PPHolonomicDriveController} bakes gains into private final PID controllers
     * with no setters, so we wrap one and rebuild it whenever any AutoPath param changes ({@code
     * isAnyChanged()} is true for exactly the loop(s) after a dashboard edit). Rebuilding drops the
     * integral accumulators, which is harmless with the kI≈0 path gains and only happens the
     * instant you retune. This makes {@link #followPath} honor NT edits mid-path, not just at
     * construction.
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
                                            AutoPoseParamsNT.translationKp.getValue(),
                                            AutoPoseParamsNT.translationKi.getValue(),
                                            AutoPoseParamsNT.translationKd.getValue()),
                                    new PIDController(
                                            AutoPoseParamsNT.rotationKp.getValue(),
                                            AutoPoseParamsNT.rotationKi.getValue(),
                                            AutoPoseParamsNT.rotationKd.getValue()),
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

    /**
     * Drive to an <b>already alliance-flipped</b> pose using the {@link AutoPilotCommand}
     * point-to-point driver instead of the pose PID ({@link #driveToPose}). Autopilot is
     * obstacle-unaware; the entry angle ({@code Params/AutoPilot/entryAngleDegrees}) shapes which
     * side the approach curve bulges toward, so tune it to route around anything in the lane — see
     * {@link AutoPilotCommand.AutoPilotParams#entryAngleDegrees}. Re-read each call (deferred), so
     * tune on the dashboard and re-run/re-press to see the new curve.
     */
    static Command driveToPoseAutoPilot(Supplier<Pose2d> targetPoseSupplier) {
        return Commands.defer(
                () -> {
                    Pose2d targetPose = targetPoseSupplier.get();
                    APTarget target =
                            new APTarget(targetPose)
                                    .withEntryAngle(
                                            Rotation2d.fromDegrees(
                                                    AutoPilotParamsNT.entryAngleDegrees
                                                            .getValue()));
                    return new AutoPilotCommand(swerve, target)
                            .beforeStarting(
                                    Commands.runOnce(
                                            () ->
                                                    Logger.recordOutput(
                                                            "Auto/targetPose", targetPose)));
                },
                Set.of(swerve));
    }

    public static Command driveToPoseAutoPilot(Pose2d targetPose) {
        return driveToPoseAutoPilot(() -> targetPose);
    }

    static Command driveToHeading(Supplier<Rotation2d> targetRotationSupplier) {
        return Commands.defer(
                () ->
                        new SwerveAimToHeading(
                                swerve,
                                RobotStateRecorder::getPoseWorldRobotCurrent,
                                targetRotationSupplier,
                                new PIDController(
                                        AutoPoseParamsNT.rotationKp.getValue(),
                                        AutoPoseParamsNT.rotationKi.getValue(),
                                        AutoPoseParamsNT.rotationKd.getValue()),
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
        // Default hold heading: crossing OUT to neutral holds the bump-start heading; coming BACK
        // holds kSlopeEnd's rotation (the existing drive-back behavior).
        Rotation2d defaultHoldHeading =
                isToNeutral
                        ? (isLeft ? kBumpStartL : kBumpStartR).getRotation()
                        : (isLeft ? kSlopeEndL : kSlopeEndR).getRotation();
        return drivePastSlope(isLeft, isToNeutral, defaultHoldHeading);
    }

    /**
     * As {@link #drivePastSlope(boolean, boolean)}, but the chassis holds {@code holdHeading} (blue
     * frame) the whole way across instead of the default. Applies to both directions: out to
     * neutral (toward the slope front) and back to the alliance side (toward the slope end). The
     * translation only sets the drive DIRECTION; the stop is the bump crossing, so the target
     * rotation is what the chassis actually holds while crossing.
     */
    public static Command drivePastSlope(
            boolean isLeft, boolean isToNeutral, Rotation2d holdHeading) {
        Pose2d anchor =
                isToNeutral
                        ? (isLeft ? kSlopeFrontL : kSlopeFrontR)
                        : (isLeft ? kSlopeEndL : kSlopeEndR);
        Pose2d target = new Pose2d(anchor.getTranslation(), holdHeading);
        return Commands.defer(
                () ->
                        Commands.deadline(
                                waitCrossedBump(isToNeutral),
                                driveToPose(AllianceFlipUtil.apply(target))),
                Set.of(swerve));
    }

    static Command waitCrossedBump(boolean isToNeutral) {
        return Commands.waitUntil(() -> hasCrossedBump(isToNeutral) && isPitchStable());
    }

    public static boolean hasCrossedBump(boolean isToNeutral) {
        return isToNeutral
                ? AllianceFlipUtil.applyX(getRobotX())
                        > FieldConstants.LinesVertical.neutralZoneNear
                : AllianceFlipUtil.applyX(getRobotX()) < 3.4;
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
     *
     * <p>The intake roller stops as soon as the shot starts. Once the upper shooter reaches its
     * target speed and the feed delay elapses, the intake enters RETRACTED_SHOOTING so the pivot
     * raises while the roller stays stopped. The pivot is actuated by {@link
     * IntakerSubsystem#followModePivot()} (running for the whole routine), which is why this only
     * flips the mode instead of commanding the pivot: the pivot requirement is already taken.
     */
    public static Command aimAndShootAtHub(double feedSeconds) {
        return Commands.sequence(
                intake.forceExtendedIdle(),
                Commands.deadline(
                        shootAtHub(feedSeconds),
                        aimAtHub(),
                        shootingSuperstructure
                                .waitForFeedStart()
                                .andThen(intake.holdRetractedShootMode())),
                // After the shot: force the intake to EXTENDED_IDLE so the hopper (whose speed is
                // driven by the intake mode) actually STOPS — the shot can otherwise leave the mode
                // in a feeding state (e.g. INTAKING carried over from the sweep). Then drop the
                // drum
                // to idle RPS (the auto sequence holds the shooter requirement, so its own default
                // idle can't run on its own).
                intake.forceExtendedIdle(),
                shootingSuperstructure.idle().withTimeout(0.02));
    }

    public static Command aimAndShootAtHub() {
        return aimAndShootAtHub(AutoShootParamsNT.feedSeconds.getValue());
    }

    /**
     * One collect/score cycle: follow {@code sweepPath} collecting; once the robot has driven out
     * past x = 7 and come back inside it (blue frame) near the end of the path, switch to feed and
     * <b>hold feed all the way through the slope crossing</b>; then empty the hopper at the hub.
     *
     * <p>The feed branch runs as a companion of {@code path + drivePastSlope} (both inside the
     * deadline), so feeding starts at x &lt; 7 and only reverts once the robot has driven past the
     * slope — not when the sweep path ends. The out-first wait stops it firing at the start, where
     * the robot begins at x ≈ 4.4 (already inside 7). {@code intake()}/{@code feed()} here only
     * flip the intake mode — they actuate the pivot because {@link
     * IntakerSubsystem#followModePivot()} runs in parallel for the whole auto (see {@link
     * AutoRoutines#competitionAuto}); inside the auto command group the pivot's own default command
     * is suppressed.
     */
    public static Command sweepCollectShoot(String sweepPath, boolean isLeft) {
        return sweepCollectShoot(sweepPath, isLeft, null);
    }

    /**
     * As {@link #sweepCollectShoot(String, boolean)}, but the drive-back across the slope holds
     * {@code driveBackHeading} (blue frame) instead of kSlopeEnd's default rotation. Pass {@code
     * null} for the default. Used e.g. to make the second sweep come back facing a specific angle.
     */
    public static Command sweepCollectShoot(
            String sweepPath, boolean isLeft, Rotation2d driveBackHeading) {
        Command driveBack =
                driveBackHeading == null
                        ? drivePastSlope(isLeft, false)
                        : drivePastSlope(isLeft, false, driveBackHeading);
        return Commands.sequence(
                Commands.deadline(
                        Commands.sequence(
                                followPathFile(sweepPath, isLeft),
                                // Pre-spin the flywheel to solution speed during the drive-back
                                // ONLY
                                // (not the outbound collect), so it's at speed on arrival without
                                // running the drum for the whole sweep. Inner deadline ends with
                                // drivePastSlope; aimAndShootAtHub then re-commands the still-
                                // spinning drum and feeds immediately. Feed only — no premature
                                // shots. If drivePastSlope is shorter than the spin-up time, the
                                // shot's readyTimeout covers the small remainder.
                                Commands.deadline(
                                        driveBack, shootingSuperstructure.spinUpForShot())),
                        intake(),
                        Commands.waitUntil(() -> AllianceFlipUtil.applyX(getRobotX()) > 7.0)
                                .andThen(
                                        Commands.waitUntil(
                                                () -> AllianceFlipUtil.applyX(getRobotX()) < 7.0))
                                .andThen(feed())),
                aimAndShootAtHub());
    }

    // =====================================================================================
    // Housekeeping
    // =====================================================================================

    /** Lift the swerve speed cap for a fast unguarded dash (e.g. the trench-start opening move). */
    public static Command setSwerveLimitUnlimited() {
        return Commands.runOnce(() -> swerve.setSwerveLimit(SwerveMK5Config.kDefaultSwerveLimit));
    }

    /** Restore the normal swerve speed cap after an unlimited-speed segment. */
    public static Command setSwerveLimitDefault() {
        return Commands.runOnce(swerve::setSwerveLimitDefault);
    }

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
