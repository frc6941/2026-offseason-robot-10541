package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoAimCommand;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.subsystems.Shooter.ShootingSuperstructure;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.commands.SwerveAimToHeading;
import lib.ironpulse.utils.AllianceFlipUtil;

public final class AutoCommands {
    public static final PathConstraints PRECISE_CONSTRAINTS =
            new PathConstraints(4.0, 2.0, Math.toRadians(360.0), Math.toRadians(540.0));
    public static final PathConstraints INTAKE_MEDIUM_CONSTRAINTS =
            new PathConstraints(4.0, 2.5, Math.toRadians(360.0), Math.toRadians(540.0));
    public static final PathConstraints TRANSIT_CONSTRAINTS =
            new PathConstraints(4.0, 4.0, Math.toRadians(540.0), Math.toRadians(720.0));
    public static final double AUTO_DURATION_SECONDS = 20.0;
    public static final double AUTO_RETURN_TIMEOUT_SECONDS = 3.0;
    public static final double AUTO_SHOOT_READY_TIMEOUT_SECONDS = 2.0;
    public static final double AUTO_SHOOT_FEED_SECONDS = 3.0;
    public static final double AUTO_MOVE_SHOT_FEED_SECONDS = 2.5;
    private static final double MOVE_SHOT_TRANSLATION_KP = 10.0;
    private static final double MOVE_SHOT_TRANSLATION_KD = 0.35;
    private static final double MOVE_SHOT_ROTATION_KP = 7.0;
    private static final double MOVE_SHOT_ROTATION_KD = 0.2;
    private static final double AUTO_TRANSLATION_TOLERANCE_METERS = 0.10;
    private static final Angle AUTO_ROTATION_TOLERANCE = Units.Degrees.of(3.0);

    public enum NeutralSweepMode {
        CONSERVATIVE,
        NEUTRAL,
        FLIGHTLESS,
        DAVIS,
        DAVIS_FRIENDSHIP,
        CORIOLIS,
        SALESMAN,
        SALESMAN_TURN
    }

    public enum NeutralSweepDirection {
        LEFT_TO_RIGHT,
        RIGHT_TO_LEFT
    }

    public enum DepotVisitRound {
        NONE,
        START,
        FIRST,
        SECOND
    }

    private AutoCommands() {}

    private static Command runWhileIntaking(Command command, IntakerSubsystem intaker) {
        return command.deadlineFor(intaker.runIntakeContinuous());
    }

    private static Command markStep(String step) {
        return Commands.runOnce(() -> SmartDashboard.putString("Auto/Step", step));
    }

    private static Command moveShotWindow(ShootingSuperstructure shootingSuperstructure) {
        return Commands.sequence(
                Commands.waitUntil(shootingSuperstructure::readyToShoot)
                        .withTimeout(AUTO_SHOOT_READY_TIMEOUT_SECONDS),
                shootingSuperstructure.feedShotForSeconds(AUTO_SHOOT_FEED_SECONDS),
                shootingSuperstructure.idle().withTimeout(0.05));
    }

    private static Command moveShotWindowShort(ShootingSuperstructure shootingSuperstructure) {
        return Commands.sequence(
                Commands.waitUntil(shootingSuperstructure::readyToShoot)
                        .withTimeout(AUTO_SHOOT_READY_TIMEOUT_SECONDS),
                shootingSuperstructure.feedShotForSeconds(AUTO_MOVE_SHOT_FEED_SECONDS),
                shootingSuperstructure.idle().withTimeout(0.05));
    }

    private static Rotation2d flipBlueHeading(Rotation2d blueHeading) {
        return AllianceFlipUtil.apply(blueHeading);
    }

    private static Rotation2d currentBlueHeading() {
        Rotation2d currentHeading = AutoBuilder.getCurrentPose().getRotation();
        return AllianceFlipUtil.shouldFlip()
                ? currentHeading.rotateBy(Rotation2d.kPi)
                : currentHeading;
    }

    private static boolean headingAtBlueGoal(Rotation2d blueHeading) {
        Rotation2d goal = flipBlueHeading(blueHeading);
        Rotation2d current = AutoBuilder.getCurrentPose().getRotation();
        double errorDeg = Math.abs(current.minus(goal).getDegrees());
        return errorDeg <= AUTO_ROTATION_TOLERANCE.in(Units.Degrees);
    }

    private static boolean translationAtBlueGoal(Translation2d blueTranslation) {
        Translation2d goal = AllianceFlipUtil.apply(blueTranslation);
        Translation2d current = AutoBuilder.getCurrentPose().getTranslation();
        return current.getDistance(goal) <= AUTO_TRANSLATION_TOLERANCE_METERS;
    }

    private static Command settleToBlueHeading(Swerve swerve, Rotation2d blueHeading) {
        return new SwerveAimToHeading(
                swerve,
                swerve::getEstimatedPose,
                () -> flipBlueHeading(blueHeading),
                new PIDController(5.0, 0.0, 0.0),
                AUTO_ROTATION_TOLERANCE,
                () -> Units.DegreesPerSecond.of(180.0));
    }

    /**
     * Strict pose move for autos:
     *
     * <ul>
     *   <li>If the robot is already at the target translation but the heading is wrong, first settle
     *       the heading in place.</li>
     *   <li>Execute a normal pathfind-to-pose step for the target pose.</li>
     *   <li>If the heading is still off when the path ends, settle the heading in place.</li>
     * </ul>
     *
     * <p>This preserves the intended semantics of each auto step as "go to this pose" without
     * introducing a separate translation-closing command that can drift or fight the pathfinder.
     */
    public static Command pathfindToBluePoseStrict(
            Swerve swerve,
            Pose2d targetPose,
            PathConstraints constraints,
            double goalEndVelocityMetersPerSecond) {
        return Commands.sequence(
                settleToBlueHeading(swerve, targetPose.getRotation())
                        .onlyIf(() -> translationAtBlueGoal(targetPose.getTranslation())
                                && !headingAtBlueGoal(targetPose.getRotation())),
                pathfindToBluePose(targetPose, constraints, goalEndVelocityMetersPerSecond),
                settleToBlueHeading(swerve, targetPose.getRotation())
                        .onlyIf(() -> !headingAtBlueGoal(targetPose.getRotation())));
    }

    public static Command pathfindToBlueTranslationWithHeadingStrict(
            Swerve swerve,
            Translation2d targetTranslation,
            Rotation2d targetHeading,
            PathConstraints constraints,
            double goalEndVelocityMetersPerSecond) {
        return pathfindToBluePoseStrict(
                swerve,
                new Pose2d(targetTranslation, targetHeading),
                constraints,
                goalEndVelocityMetersPerSecond);
    }

    /**
     * Pathfind to a target pose using the default goal end velocity behavior from PathPlanner.
     *
     * <p>Example usage:
     *
     * <pre>{@code
     * Pose2d targetPose = new Pose2d(4.0, 2.0, Rotation2d.fromDegrees(180));
     * PathConstraints constraints = new PathConstraints(
     *     2.0,
     *     2.0,
     *     Math.toRadians(360),
     *     Math.toRadians(540));
     *
     * Command cmd = AutoCommands.pathfindToPose(targetPose, constraints);
     * }</pre>
     */
    // --- Auto preview capture ---
    // While a sink is set, every blue-frame pathfind target is recorded so the chooser can draw the
    // selected auto's waypoints (Field2d "AutoPreview") without running it. Build-time only — no
    // effect on the scheduled command. All blue pathfind variants funnel through pathfindToBluePose.
    private static java.util.List<Pose2d> previewSink = null;

    public static void startPreviewCapture(java.util.List<Pose2d> sink) {
        previewSink = sink;
    }

    public static void stopPreviewCapture() {
        previewSink = null;
    }

    private static void capturePreviewTarget(Pose2d targetPose) {
        if (previewSink != null) {
            previewSink.add(targetPose);
        }
    }

    public static Command pathfindToPose(Pose2d targetPose, PathConstraints constraints) {
        return AutoBuilder.pathfindToPose(targetPose, constraints);
    }

    /**
     * Pathfind to a blue-alliance pose and let PathPlanner flip it automatically for the current
     * alliance.
     */
    public static Command pathfindToBluePose(Pose2d targetPose, PathConstraints constraints) {
        capturePreviewTarget(targetPose);
        return AutoBuilder.pathfindToPoseFlipped(targetPose, constraints);
    }

    /**
     * Pathfind to a blue-alliance pose with an explicit goal end velocity and let PathPlanner flip
     * it automatically for the current alliance.
     */
    public static Command pathfindToBluePose(
            Pose2d targetPose,
            PathConstraints constraints,
            double goalEndVelocityMetersPerSecond) {
        capturePreviewTarget(targetPose);
        return AutoBuilder.pathfindToPoseFlipped(
                targetPose, constraints, goalEndVelocityMetersPerSecond);
    }

    public static Command pathfindToBlueTranslationWithHeading(
            Translation2d targetTranslation,
            Rotation2d targetHeading,
            PathConstraints constraints,
            double goalEndVelocityMetersPerSecond) {
        return pathfindToBluePose(
                new Pose2d(targetTranslation, targetHeading),
                constraints,
                goalEndVelocityMetersPerSecond);
    }

    public static Command pathfindToBlueTranslationPreserveHeading(
            Translation2d targetTranslation,
            PathConstraints constraints,
            double goalEndVelocityMetersPerSecond) {
        return pathfindToBluePose(
                new Pose2d(targetTranslation, currentBlueHeading()),
                constraints,
                goalEndVelocityMetersPerSecond);
    }

    /**
     * Pathfind to a target pose with an explicit goal end velocity in meters per second.
     *
     * <p>Example usage:
     *
     * <pre>{@code
     * Pose2d targetPose = new Pose2d(4.0, 2.0, Rotation2d.fromDegrees(180));
     * PathConstraints constraints = new PathConstraints(
     *     2.0,
     *     2.0,
     *     Math.toRadians(360),
     *     Math.toRadians(540));
     *
     * Command cmd = AutoCommands.pathfindToPose(targetPose, constraints, 0.0);
     * }</pre>
     */
    public static Command pathfindToPose(
            Pose2d targetPose,
            PathConstraints constraints,
            double goalEndVelocityMetersPerSecond) {
        return AutoBuilder.pathfindToPose(targetPose, constraints, goalEndVelocityMetersPerSecond);
    }

    /**
     * Pathfind to a target pose with a WPILib units-based goal end velocity.
     *
     * <p>Example usage:
     *
     * <pre>{@code
     * Pose2d targetPose = new Pose2d(4.0, 2.0, Rotation2d.fromDegrees(180));
     * PathConstraints constraints = new PathConstraints(
     *     2.0,
     *     2.0,
     *     Math.toRadians(360),
     *     Math.toRadians(540));
     *
     * Command cmd = AutoCommands.pathfindToPose(
     *     targetPose,
     *     constraints,
     *     MetersPerSecond.of(0.0));
     * }</pre>
     */
    public static Command pathfindToPose(
            Pose2d targetPose,
            PathConstraints constraints,
            LinearVelocity goalEndVelocity) {
        return AutoBuilder.pathfindToPose(targetPose, constraints, goalEndVelocity);
    }

    private static Rotation2d neutralSweepHeading(NeutralSweepDirection direction) {
        return direction == NeutralSweepDirection.LEFT_TO_RIGHT
                ? Rotation2d.fromDegrees(-90.0)
                : Rotation2d.fromDegrees(90.0);
    }

    private static Translation2d[] reverse(Translation2d[] points) {
        Translation2d[] reversed = new Translation2d[points.length];
        for (int i = 0; i < points.length; i++) {
            reversed[i] = points[points.length - 1 - i];
        }
        return reversed;
    }

    private static Translation2d[] neutralSweepPoints(
            NeutralSweepMode mode,
            NeutralSweepDirection direction) {
        Translation2d[] leftToRightPoints = switch (mode) {
            case CONSERVATIVE -> new Translation2d[] {
                AutoPoints.NeutralZone.LEFT_CONSERVATIVE,
                AutoPoints.NeutralZone.RIGHT_CONSERVATIVE
            };
            case NEUTRAL -> new Translation2d[] {
                AutoPoints.NeutralZone.LEFT_CENTER,
                AutoPoints.NeutralZone.RIGHT_CENTER
            };
            case FLIGHTLESS -> new Translation2d[] {
                AutoPoints.NeutralZone.LEFT_FLIGHTLESS,
                AutoPoints.NeutralZone.RIGHT_FLIGHTLESS
            };
            case DAVIS -> new Translation2d[] {
                AutoPoints.NeutralZone.LEFT_DAVIS,
                AutoPoints.NeutralZone.RIGHT_DAVIS
            };
            case DAVIS_FRIENDSHIP -> new Translation2d[] {
                AutoPoints.NeutralZone.LEFT_DAVIS,
                AutoPoints.NeutralZone.LEFT_CENTER,
                AutoPoints.NeutralZone.RIGHT_CENTER,
                AutoPoints.NeutralZone.RIGHT_DAVIS
            };
            case CORIOLIS -> new Translation2d[] {
                AutoPoints.NeutralZone.LEFT_CORIOLIS,
                AutoPoints.NeutralZone.RIGHT_CORIOLIS
            };
            case SALESMAN -> new Translation2d[] {
                AutoPoints.NeutralZone.LEFT_EDGE,
                AutoPoints.NeutralZone.LEFT_CENTER,
                AutoPoints.NeutralZone.RIGHT_CENTER,
                AutoPoints.NeutralZone.RIGHT_EDGE
            };
            case SALESMAN_TURN -> new Translation2d[] {
                AutoPoints.NeutralZone.LEFT_SALESMAN_TURN,
                AutoPoints.NeutralZone.LEFT_CENTER,
                AutoPoints.NeutralZone.RIGHT_CENTER,
                AutoPoints.NeutralZone.RIGHT_SALESMAN_TURN
            };
        };

        return direction == NeutralSweepDirection.LEFT_TO_RIGHT
                ? leftToRightPoints
                : reverse(leftToRightPoints);
    }

    public static Command neutralZoneSweep(
            Swerve swerve,
            IntakerSubsystem intaker,
            NeutralSweepMode mode,
            NeutralSweepDirection direction) {
        Rotation2d heading = neutralSweepHeading(direction);
        Translation2d[] points = neutralSweepPoints(mode, direction);
        Translation2d start = points[0];
        Translation2d end = points[points.length - 1];

        Command sweep = Commands.sequence(
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        start,
                        heading,
                        PRECISE_CONSTRAINTS,
                        0.0),
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        end,
                        heading,
                        INTAKE_MEDIUM_CONSTRAINTS,
                        0.0));

        return runWhileIntaking(sweep, intaker);
    }

    public static Command autoAimAndShoot(
            Swerve swerve,
            ShootingSuperstructure shootingSuperstructure) {
        return Commands.deadline(
                moveShotWindow(shootingSuperstructure),
                new AutoAimCommand(
                        swerve,
                        () -> 0.0,
                        () -> 0.0,
                        shootingSuperstructure::aimHeading,
                        shootingSuperstructure::aimHeadingRateRadPerSec));
    }

    public static Command driveAndShootToBlueTranslation(
            Swerve swerve,
            ShootingSuperstructure shootingSuperstructure,
            Translation2d targetTranslation,
            Distance translationTolerance,
            Angle rotationTolerance) {
        capturePreviewTarget(new Pose2d(targetTranslation, Rotation2d.kZero));
        return Commands.parallel(
                SwerveCommands.driveToPose(
                        swerve,
                        swerve::getEstimatedPose,
                        () -> new Pose3d(
                                new Pose2d(
                                        AllianceFlipUtil.apply(targetTranslation),
                                        shootingSuperstructure.aimHeading())),
                        () -> Pose2d.kZero,
                        new PIDController(MOVE_SHOT_TRANSLATION_KP, 0.0, MOVE_SHOT_TRANSLATION_KD),
                        new PIDController(MOVE_SHOT_ROTATION_KP, 0.0, MOVE_SHOT_ROTATION_KD),
                        translationTolerance,
                        rotationTolerance)
                        .withTimeout(AUTO_RETURN_TIMEOUT_SECONDS),
                moveShotWindowShort(shootingSuperstructure));
    }

    private static Pose2d depotStartPose(AutoSelector.DepotAxis depotAxis) {
        return depotAxis == AutoSelector.DepotAxis.X
                ? AutoPoints.DEPOT_X_START
                : AutoPoints.DEPOT_Y_START;
    }

    private static Command depotCollectFromSelection(
            Swerve swerve,
            IntakerSubsystem intaker,
            AutoSelector.DepotAxis depotAxis) {
        return depotAxis == AutoSelector.DepotAxis.X
                ? depotXCollect(swerve, intaker)
                : depotYCollect(swerve, intaker);
    }

    private static Command shootThenDepotCollect(
            Swerve swerve,
            IntakerSubsystem intaker,
            ShootingSuperstructure shootingSuperstructure,
            AutoSelector.DepotAxis depotAxis,
            String stepPrefix) {
        Pose2d depotStart = depotStartPose(depotAxis);
        return Commands.sequence(
                markStep(stepPrefix + ": move-shot to depot start"),
                driveAndShootToBlueTranslation(
                        swerve,
                        shootingSuperstructure,
                        depotStart.getTranslation(),
                        Units.Meters.of(0.15),
                        Units.Degrees.of(5.0)),
                markStep(stepPrefix + ": depot collect"),
                depotCollectFromSelection(swerve, intaker, depotAxis));
    }

    private static Command handlePostSweepAction(
            Swerve swerve,
            IntakerSubsystem intaker,
            ShootingSuperstructure shootingSuperstructure,
            NeutralSweepDirection sweepDirection,
            AutoSelector.DepotAxis depotAxis,
            DepotVisitRound depotRound,
            int roundIndex) {
        DepotVisitRound currentRound = roundIndex == 1 ? DepotVisitRound.FIRST : DepotVisitRound.SECOND;

        if (depotRound == currentRound) {
            return Commands.sequence(
                    markStep("Round " + roundIndex + ": launch pose"),
                    goToBumpLaunchForSweepEnd(swerve, sweepDirection)
                            .withTimeout(AUTO_RETURN_TIMEOUT_SECONDS),
                    shootThenDepotCollect(swerve, intaker, shootingSuperstructure, depotAxis, "Round " + roundIndex));
        }

        return Commands.sequence(
                markStep("Round " + roundIndex + ": launch pose"),
                goToBumpLaunchForSweepEnd(swerve, roundIndex == 1
                        ? NeutralSweepDirection.LEFT_TO_RIGHT
                        : NeutralSweepDirection.RIGHT_TO_LEFT)
                        .withTimeout(AUTO_RETURN_TIMEOUT_SECONDS),
                markStep("Round " + roundIndex + ": shoot"),
                autoAimAndShoot(swerve, shootingSuperstructure));
    }

    public static Command goToBumpLaunchForSweepEnd(
            Swerve swerve,
            NeutralSweepDirection direction) {
        return direction == NeutralSweepDirection.LEFT_TO_RIGHT
                ? pathfindToBlueTranslationPreserveHeading(
                        AutoPoints.Launch.RIGHT_BUMP.getTranslation(),
                        TRANSIT_CONSTRAINTS,
                        0.0)
                : pathfindToBlueTranslationPreserveHeading(
                        AutoPoints.Launch.LEFT_BUMP.getTranslation(),
                        TRANSIT_CONSTRAINTS,
                        0.0);
    }

    public static Command midTwoCycle(
            Swerve swerve,
            IntakerSubsystem intaker,
            ShootingSuperstructure shootingSuperstructure,
            NeutralSweepMode firstMode,
            NeutralSweepMode secondMode,
            NeutralSweepDirection firstDirection,
            AutoSelector.DepotAxis depotAxis,
            DepotVisitRound depotRound) {
        NeutralSweepDirection secondDirection = firstDirection == NeutralSweepDirection.LEFT_TO_RIGHT
                ? NeutralSweepDirection.RIGHT_TO_LEFT
                : NeutralSweepDirection.LEFT_TO_RIGHT;

        return Commands.sequence(
                Commands.either(
                        Commands.sequence(
                                markStep("Start: depot collect"),
                                depotCollectFromSelection(swerve, intaker, depotAxis)),
                        Commands.none(),
                        () -> depotRound == DepotVisitRound.START),
                markStep("Mid Two Cycle: first intake"),
                neutralZoneSweep(swerve, intaker, firstMode, firstDirection),
                handlePostSweepAction(
                        swerve,
                        intaker,
                        shootingSuperstructure,
                        firstDirection,
                        depotAxis,
                        depotRound,
                        1),
                markStep("Mid Two Cycle: second intake"),
                neutralZoneSweep(swerve, intaker, secondMode, secondDirection),
                handlePostSweepAction(
                        swerve,
                        intaker,
                        shootingSuperstructure,
                        secondDirection,
                        depotAxis,
                        depotRound,
                        2))
                .withTimeout(AUTO_DURATION_SECONDS);
    }

    /** DEPOT X START -> intake on -> DEPOT X END */
    public static Command depotXCollect(Swerve swerve, IntakerSubsystem intaker) {
        return Commands.sequence(
                pathfindToBluePoseStrict(swerve, AutoPoints.DEPOT_X_START, PRECISE_CONSTRAINTS, 0.0),
                runWhileIntaking(
                        pathfindToBluePoseStrict(
                                swerve,
                                AutoPoints.DEPOT_X_END,
                                INTAKE_MEDIUM_CONSTRAINTS,
                                0.0),
                        intaker));
    }

    /** DEPOT Y START -> intake on -> DEPOT Y END */
    public static Command depotYCollect(Swerve swerve, IntakerSubsystem intaker) {
        return Commands.sequence(
                pathfindToBluePoseStrict(swerve, AutoPoints.DEPOT_Y_START, PRECISE_CONSTRAINTS, 0.0),
                runWhileIntaking(
                        pathfindToBluePoseStrict(
                                swerve,
                                AutoPoints.DEPOT_Y_END,
                                INTAKE_MEDIUM_CONSTRAINTS,
                                0.0),
                        intaker));
    }

    /** Move to the OUTPOST pose. */
    public static Command goToOutpost(Swerve swerve) {
        return pathfindToBluePoseStrict(swerve, AutoPoints.OUTPOST, TRANSIT_CONSTRAINTS, 0.0);
    }

    public static Command goToHubCenterStart(Swerve swerve) {
        return pathfindToBlueTranslationWithHeadingStrict(
                swerve,
                AutoPoints.Hub.CENTER_START,
                Rotation2d.kPi,
                TRANSIT_CONSTRAINTS,
                0.0);
    }

    public static Command trenchLeftStartToClear(Swerve swerve) {
        return Commands.sequence(
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.Trench.LEFT_START,
                        Rotation2d.kPi,
                        PRECISE_CONSTRAINTS,
                        0.0),
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.Trench.LEFT_CLEAR,
                        Rotation2d.kPi,
                        TRANSIT_CONSTRAINTS,
                        0.0));
    }

    public static Command trenchRightStartToClear(Swerve swerve) {
        return Commands.sequence(
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.Trench.RIGHT_START,
                        Rotation2d.kPi,
                        PRECISE_CONSTRAINTS,
                        0.0),
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.Trench.RIGHT_CLEAR,
                        Rotation2d.kPi,
                        TRANSIT_CONSTRAINTS,
                        0.0));
    }

    public static Command bumpLeftInnerToOuter(Swerve swerve) {
        return Commands.sequence(
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.Bump.LEFT_INNER,
                        Rotation2d.kPi,
                        PRECISE_CONSTRAINTS,
                        0.0),
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.Bump.LEFT_OUTER,
                        Rotation2d.kPi,
                        TRANSIT_CONSTRAINTS,
                        0.0));
    }

    public static Command bumpRightInnerToOuter(Swerve swerve) {
        return Commands.sequence(
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.Bump.RIGHT_INNER,
                        Rotation2d.kPi,
                        PRECISE_CONSTRAINTS,
                        0.0),
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.Bump.RIGHT_OUTER,
                        Rotation2d.kPi,
                        TRANSIT_CONSTRAINTS,
                        0.0));
    }

    public static Command depotLeftThrough(Swerve swerve, IntakerSubsystem intaker) {
        return runWhileIntaking(
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.Depot.LEFT_THROUGH,
                        Rotation2d.kZero,
                        INTAKE_MEDIUM_CONSTRAINTS,
                        0.0),
                intaker);
    }

    public static Command depotRightThrough(Swerve swerve, IntakerSubsystem intaker) {
        return runWhileIntaking(
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.Depot.RIGHT_THROUGH,
                        Rotation2d.kZero,
                        INTAKE_MEDIUM_CONSTRAINTS,
                        0.0),
                intaker);
    }

    public static Command towerLeftThrough(Swerve swerve) {
        return pathfindToBlueTranslationWithHeadingStrict(
                swerve,
                AutoPoints.Tower.LEFT_THROUGH,
                Rotation2d.kZero,
                PRECISE_CONSTRAINTS,
                0.0);
    }

    public static Command towerRightThrough(Swerve swerve) {
        return pathfindToBlueTranslationWithHeadingStrict(
                swerve,
                AutoPoints.Tower.RIGHT_THROUGH,
                Rotation2d.kZero,
                PRECISE_CONSTRAINTS,
                0.0);
    }

    public static Command goToLeftBumpLaunch(Swerve swerve) {
        return pathfindToBluePoseStrict(swerve, AutoPoints.Launch.LEFT_BUMP, TRANSIT_CONSTRAINTS, 0.0);
    }

    public static Command goToRightBumpLaunch(Swerve swerve) {
        return pathfindToBluePoseStrict(swerve, AutoPoints.Launch.RIGHT_BUMP, TRANSIT_CONSTRAINTS, 0.0);
    }

    public static Command goToLeftTrenchLaunch(Swerve swerve) {
        return pathfindToBluePoseStrict(swerve, AutoPoints.Launch.LEFT_TRENCH, TRANSIT_CONSTRAINTS, 0.0);
    }

    public static Command goToRightTrenchLaunch(Swerve swerve) {
        return pathfindToBluePoseStrict(swerve, AutoPoints.Launch.RIGHT_TRENCH, TRANSIT_CONSTRAINTS, 0.0);
    }

    public static Command goToLeftClimb(Swerve swerve) {
        return pathfindToBluePoseStrict(swerve, AutoPoints.Climb.LEFT, PRECISE_CONSTRAINTS, 0.0);
    }

    public static Command goToRightClimb(Swerve swerve) {
        return pathfindToBluePoseStrict(swerve, AutoPoints.Climb.RIGHT, PRECISE_CONSTRAINTS, 0.0);
    }

    /** Sweep the middle line from left to right while facing -90 degrees and intaking. */
    public static Command sweepMidLeftToRight(Swerve swerve, IntakerSubsystem intaker) {
        return neutralZoneSweep(
                swerve,
                intaker,
                NeutralSweepMode.SALESMAN,
                NeutralSweepDirection.LEFT_TO_RIGHT);
    }

    /** Sweep the middle line from right to left while facing 90 degrees and intaking. */
    public static Command sweepMidRightToLeft(Swerve swerve, IntakerSubsystem intaker) {
        return neutralZoneSweep(
                swerve,
                intaker,
                NeutralSweepMode.SALESMAN,
                NeutralSweepDirection.RIGHT_TO_LEFT);
    }
}
