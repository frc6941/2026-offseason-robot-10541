package frc.robot.auto;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotStateRecorder;
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
 * and follows it with a {@link FollowPathCommand} (NOT the pathfinding / setpoint-generator
 * approach). {@link #driveToPose} / {@link #driveToHeading} handle precise point moves.
 *
 * <p><b>Key difference from the competition robot:</b> that robot has a turret and can shoot while
 * moving; this robot cannot. So instead of a "shoot anytime" action, autonomous drives to a fixed
 * <b>shoot pose</b> (already facing the hub), holds still, and empties the hopper — see {@link
 * #driveToShootAndFire}.
 *
 * <p>All poses below are authored in the <b>blue</b> field frame; wrap with {@link
 * AllianceFlipUtil#apply} at use so they work for both alliances. These are PLACEHOLDERS — measure
 * and edit them for the real field.
 */
public class AutoActions {
    // ---- Shoot poses: robot sits here, already aimed at the hub, and empties the hopper. ----
    public static final Pose2d kShootL = new Pose2d(3.0, 5.35, Rotation2d.fromDegrees(-45));
    public static final Pose2d kShootR = new Pose2d(3.0, 2.56, Rotation2d.fromDegrees(45));

    // ---- Intake / depot alignment poses. ----
    public static final Pose2d kDepotIntake = new Pose2d(1.013, 5.987, Rotation2d.fromDegrees(180));
    public static final Pose2d kStationIntake = new Pose2d(0.59, 0.72, Rotation2d.fromDegrees(180));

    // ---- Start poses (used by resetOnPose in sim). ----
    public static final Pose2d kStartL = new Pose2d(7.54, 5.35, Rotation2d.fromDegrees(0));
    public static final Pose2d kStartR = new Pose2d(7.54, 2.56, Rotation2d.fromDegrees(0));

    // ---- Test path waypoints. ----
    public static final Pose2d kTestA = new Pose2d(1.509, 6.14, Rotation2d.fromDegrees(6.3));
    public static final Pose2d kTestB = new Pose2d(2.79, 5.2, Rotation2d.fromDegrees(-72));
    public static final Pose2d kTestC = new Pose2d(2.69, 3.07, Rotation2d.fromDegrees(-110));

    private static Swerve swerve;
    private static ShootingSuperstructure shootingSuperstructure;
    private static IntakerSubsystem intake;

    public static void init(
            Swerve swerve, ShootingSuperstructure shootingSuperstructure, IntakerSubsystem intake) {
        AutoActions.swerve = swerve;
        AutoActions.shootingSuperstructure = shootingSuperstructure;
        AutoActions.intake = intake;
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
                        new PPHolonomicDriveController(
                                new PIDConstants(
                                        AutoParams.AutoPathParams.kpStrave,
                                        AutoParams.AutoPathParams.kiStrave,
                                        AutoParams.AutoPathParams.kdStrave),
                                new PIDConstants(
                                        AutoParams.AutoPathParams.kpSpin,
                                        AutoParams.AutoPathParams.kiSpin,
                                        AutoParams.AutoPathParams.kdSpin),
                                RobotConstants.LOOPER_DT),
                        RobotConstants.AUTO_ROBOT_CONFIG,
                        () -> false, // already flipped/mirrored before we get here
                        swerve)
                .beforeStarting(
                        () ->
                                Logger.recordOutput(
                                        "Auto/Traj", path.getPathPoses().toArray(new Pose2d[0])));
    }

    /**
     * Load a {@code .path} file from {@code deploy/pathplanner/paths}, optionally mirror it (for
     * the left/right symmetric variant), flip it for the current alliance, then follow it.
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
                                            AutoParams.AutoPoseParams.kpStrave,
                                            AutoParams.AutoPoseParams.kiStrave,
                                            AutoParams.AutoPoseParams.kdStrave),
                                    new PIDController(
                                            AutoParams.AutoPoseParams.kpSpin,
                                            AutoParams.AutoPoseParams.kiSpin,
                                            AutoParams.AutoPoseParams.kdSpin),
                                    Meters.of(AutoParams.AutoPoseParams.tolerancePositionM),
                                    Degrees.of(AutoParams.AutoPoseParams.toleranceHeadingDeg))
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
                                        AutoParams.AutoPoseParams.kpSpin,
                                        AutoParams.AutoPoseParams.kiSpin,
                                        AutoParams.AutoPoseParams.kdSpin),
                                Degrees.of(AutoParams.AutoPoseParams.toleranceHeadingDeg),
                                () -> DegreesPerSecond.of(80)),
                Set.of(swerve));
    }

    // =====================================================================================
    // Intake actions
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

    // =====================================================================================
    // Shooting actions (non-turret: drive to a pose, hold still, empty the hopper)
    // =====================================================================================

    /** Hold the current spot with the wheels X-locked so we don't drift while shooting. */
    public static Command holdStill() {
        return Commands.run(swerve::runStopAndLock, swerve);
    }

    /** Spin up and feed for {@code feedSeconds}; sized to empty a full hopper. */
    public static Command shootAllBalls(double feedSeconds) {
        return shootingSuperstructure.shootWhenReadyForSeconds(
                AutoParams.AutoShootParams.readyTimeoutSeconds, feedSeconds);
    }

    public static Command shootAllBalls() {
        return shootAllBalls(AutoParams.AutoShootParams.feedSeconds);
    }

    /**
     * The core non-turret scoring move: drive to {@code blueShootPose} (which already faces the
     * hub), then hold position and empty the hopper.
     */
    public static Command driveToShootAndFire(Pose2d blueShootPose, double feedSeconds) {
        return Commands.sequence(
                driveToPose(AllianceFlipUtil.apply(blueShootPose)),
                Commands.deadline(shootAllBalls(feedSeconds), holdStill()));
    }

    public static Command driveToShootAndFire(Pose2d blueShootPose) {
        return driveToShootAndFire(blueShootPose, AutoParams.AutoShootParams.feedSeconds);
    }

    /** Convenience: drive to the left or right shoot pose and empty the hopper. */
    public static Command driveToShootAndFire(boolean isLeft) {
        return driveToShootAndFire(isLeft ? kShootL : kShootR);
    }

    // =====================================================================================
    // Housekeeping
    // =====================================================================================

    public static Command zeroEverything() {
        return Commands.parallel(shootingSuperstructure.zeroHoodHere(), intake.zeroCommand());
    }

    /** In sim only, snap the pose estimator + transform tree to a known start pose. */
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

    /** Example on-the-fly path (mirrors the competition robot's {@code testPath}). */
    public static Command testPath() {
        return Commands.defer(
                () -> {
                    Pose2d current = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
                    List<Pose2d> waypoints =
                            List.of(
                                    current,
                                    AllianceFlipUtil.apply(kTestA),
                                    AllianceFlipUtil.apply(kTestB),
                                    AllianceFlipUtil.apply(kTestC));
                    PathPlannerPath path =
                            generatePath(waypoints, Collections.emptyList(), 4.2, 6.0, 0.0);
                    return followPath(path);
                },
                Set.of(swerve));
    }

    static double getRobotX() {
        return RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d().getX();
    }

    static double getRobotY() {
        return RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d().getY();
    }

    static boolean hasCrossedNeutralLine() {
        return AllianceFlipUtil.applyX(getRobotX()) > FieldConstants.LinesVertical.neutralZoneNear;
    }
}
