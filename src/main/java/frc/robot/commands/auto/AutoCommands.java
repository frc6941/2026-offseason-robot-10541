package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoAimParamsNT;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.subsystems.Shooter.ShootingSuperstructure;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.utils.AllianceFlipUtil;

public final class AutoCommands {
    private static final PIDController rotationController =
            new PIDController(AutoAimParamsNT.kP.getValue(), 0.0, 0.0);

    public enum NeutralSweepMode {
        CONSERVATIVE,
        NEUTRAL,
        FLIGHTLESS,
        FLIGHTLESS_WIDE,
        DAVIS,
        DAVIS_FRIENDSHIP,
        CORIOLIS,
        CENTER_FORWARD,
        SALESMAN,
        SALESMAN_TURN,
        WAVE,
        FLIGHTLESS_WAVE
    }

    public enum NeutralSweepDirection {
        LEFT_TO_RIGHT,
        RIGHT_TO_LEFT
    }

    public enum MidKind {
        FULL,
        HALF
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

    private static Command moveShotWindow(
            ShootingSuperstructure shootingSuperstructure, IntakerSubsystem intaker) {
        return Commands.deadline(
                Commands.sequence(
                        shootingSuperstructure.shootWhenReadyForSeconds(
                                AutoCommandParamsNT.autoShootReadyTimeoutSeconds.getValue(),
                                AutoCommandParamsNT.autoShootFeedSeconds.getValue()),
                        shootingSuperstructure.idle().withTimeout(0.05)),
                intaker.holdRetractedFeedPosition());
    }

    private static Command moveShotWindowShort(
            ShootingSuperstructure shootingSuperstructure, IntakerSubsystem intaker) {
        return Commands.deadline(
                Commands.sequence(
                        shootingSuperstructure.shootWhenReadyForSeconds(
                                AutoCommandParamsNT.autoShootReadyTimeoutSeconds.getValue(),
                                AutoCommandParamsNT.autoMoveShotFeedSeconds.getValue()),
                        shootingSuperstructure.idle().withTimeout(0.05)),
                intaker.holdRetractedFeedPosition());
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
        return errorDeg <= AutoCommandParamsNT.autoRotationToleranceDegrees.getValue();
    }

    private static boolean translationAtBlueGoal(Translation2d blueTranslation) {
        Translation2d goal = AllianceFlipUtil.apply(blueTranslation);
        Translation2d current = AutoBuilder.getCurrentPose().getTranslation();
        return current.getDistance(goal)
                <= AutoCommandParamsNT.autoTranslationToleranceMeters.getValue();
    }

    private static boolean poseAtGoal(
            Pose2d currentPose,
            Pose2d targetPose,
            Distance translationTolerance,
            Angle rotationTolerance) {
        return currentPose.getTranslation().getDistance(targetPose.getTranslation())
                        <= translationTolerance.in(Units.Meters)
                && Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees())
                        <= rotationTolerance.in(Units.Degrees);
    }

    private static Command settleToBlueHeading(Swerve swerve, Rotation2d blueHeading) {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        return Commands.run(
                        () -> {
                            Rotation2d goalHeading = flipBlueHeading(blueHeading);
                            Rotation2d currentHeading = AutoBuilder.getCurrentPose().getRotation();
                            double omega =
                                    rotationController.calculate(
                                            currentHeading.getRadians(), goalHeading.getRadians());
                            omega =
                                    MathUtil.clamp(
                                            omega,
                                            -AutoAimCommand.AutoAimParams.maxAngularVelRadPerSec,
                                            AutoAimCommand.AutoAimParams.maxAngularVelRadPerSec);
                            swerve.runTwist(new ChassisSpeeds(0.0, 0.0, omega));
                        },
                        swerve)
                .beforeStarting(rotationController::reset)
                .until(() -> headingAtBlueGoal(blueHeading))
                .finallyDo(interrupted -> swerve.runStop());
    }

    private static Command driveToDynamicPose(
            Swerve swerve,
            Supplier<Pose2d> targetPoseSupplier,
            PIDController translationController,
            PIDController rotationController,
            Distance translationTolerance,
            Angle rotationTolerance) {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        return Commands.run(
                        () -> {
                            Pose2d currentPose = AutoBuilder.getCurrentPose();
                            Pose2d targetPose = targetPoseSupplier.get();
                            Pose2d targetRobot = targetPose.relativeTo(currentPose);

                            Translation2d translationError = targetRobot.getTranslation();
                            double translationErrorNorm = translationError.getNorm();
                            Rotation2d translationDirection =
                                    translationErrorNorm <= 1e-6
                                            ? Rotation2d.kZero
                                            : translationError.getAngle();
                            double translationCommand =
                                    translationController.calculate(translationErrorNorm, 0.0);
                            Translation2d velocityRobot =
                                    new Translation2d(-translationCommand, translationDirection);

                            double omega =
                                    -rotationController.calculate(
                                            targetRobot.getRotation().getRadians(), 0.0);
                            omega =
                                    MathUtil.clamp(
                                            omega,
                                            -AutoAimCommand.AutoAimParams.maxAngularVelRadPerSec,
                                            AutoAimCommand.AutoAimParams.maxAngularVelRadPerSec);

                            swerve.runTwist(
                                    new ChassisSpeeds(
                                            velocityRobot.getX(), velocityRobot.getY(), omega));
                        },
                        swerve)
                .beforeStarting(
                        () -> {
                            translationController.reset();
                            rotationController.reset();
                        })
                .until(
                        () ->
                                poseAtGoal(
                                        AutoBuilder.getCurrentPose(),
                                        targetPoseSupplier.get(),
                                        translationTolerance,
                                        rotationTolerance))
                .finallyDo(interrupted -> swerve.runStop());
    }

    /**
     * Strict pose move for autos:
     *
     * <ul>
     *   <li>If the robot is already at the target translation but the heading is wrong, first
     *       settle the heading in place.
     *   <li>Execute a normal pathfind-to-pose step for the target pose.
     *   <li>If the heading is still off when the path ends, settle the heading in place.
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
                        .onlyIf(
                                () ->
                                        translationAtBlueGoal(targetPose.getTranslation())
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
    // effect on the scheduled command. All blue pathfind variants funnel through
    // pathfindToBluePose.
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

    private static void capturePreviewPath(PathPlannerPath path) {
        if (previewSink != null) {
            previewSink.addAll(path.getPathPoses());
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
            Pose2d targetPose, PathConstraints constraints, double goalEndVelocityMetersPerSecond) {
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
            Pose2d targetPose, PathConstraints constraints, double goalEndVelocityMetersPerSecond) {
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
            Pose2d targetPose, PathConstraints constraints, LinearVelocity goalEndVelocity) {
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
            NeutralSweepMode mode, NeutralSweepDirection direction) {
        Translation2d[] leftToRightPoints =
                switch (mode) {
                    case CONSERVATIVE ->
                            new Translation2d[] {
                                AutoPoints.NeutralZone.LEFT_CONSERVATIVE,
                                AutoPoints.NeutralZone.RIGHT_CONSERVATIVE
                            };
                    case NEUTRAL ->
                            new Translation2d[] {
                                AutoPoints.NeutralZone.LEFT_CENTER,
                                AutoPoints.NeutralZone.RIGHT_CENTER
                            };
                    case FLIGHTLESS ->
                            new Translation2d[] {
                                AutoPoints.NeutralZone.LEFT_FLIGHTLESS,
                                AutoPoints.NeutralZone.RIGHT_FLIGHTLESS
                            };
                    case FLIGHTLESS_WIDE ->
                            new Translation2d[] {
                                AutoPoints.NeutralZone.LEFT_FLIGHTLESS_WIDE,
                                AutoPoints.NeutralZone.RIGHT_FLIGHTLESS_WIDE
                            };
                    case FLIGHTLESS_WAVE ->
                            new Translation2d[] {
                                AutoPoints.NeutralZone.LEFT_FLIGHTLESS,
                                AutoPoints.NeutralZone.RIGHT_FLIGHTLESS
                            };
                    case DAVIS ->
                            new Translation2d[] {
                                AutoPoints.NeutralZone.LEFT_DAVIS,
                                AutoPoints.NeutralZone.RIGHT_DAVIS
                            };
                    case DAVIS_FRIENDSHIP ->
                            new Translation2d[] {
                                AutoPoints.NeutralZone.LEFT_DAVIS,
                                AutoPoints.NeutralZone.LEFT_CENTER,
                                AutoPoints.NeutralZone.RIGHT_CENTER,
                                AutoPoints.NeutralZone.RIGHT_DAVIS
                            };
                    case CORIOLIS ->
                            new Translation2d[] {
                                AutoPoints.NeutralZone.LEFT_CORIOLIS,
                                AutoPoints.NeutralZone.RIGHT_CORIOLIS
                            };
                    case CENTER_FORWARD ->
                            new Translation2d[] {
                                AutoPoints.NeutralZone.LEFT_CENTER_FORWARD,
                                AutoPoints.NeutralZone.RIGHT_CENTER_FORWARD
                            };
                    case SALESMAN ->
                            new Translation2d[] {
                                AutoPoints.NeutralZone.LEFT_EDGE,
                                AutoPoints.NeutralZone.LEFT_CENTER,
                                AutoPoints.NeutralZone.RIGHT_CENTER,
                                AutoPoints.NeutralZone.RIGHT_EDGE
                            };
                    case SALESMAN_TURN ->
                            new Translation2d[] {
                                AutoPoints.NeutralZone.LEFT_SALESMAN_TURN,
                                AutoPoints.NeutralZone.LEFT_CENTER,
                                AutoPoints.NeutralZone.RIGHT_CENTER,
                                AutoPoints.NeutralZone.RIGHT_SALESMAN_TURN
                            };
                    case WAVE ->
                            new Translation2d[] {
                                AutoPoints.NeutralZone.LEFT_CENTER,
                                AutoPoints.NeutralZone.RIGHT_CENTER
                            };
                };

        return direction == NeutralSweepDirection.LEFT_TO_RIGHT
                ? leftToRightPoints
                : reverse(leftToRightPoints);
    }

    private static boolean isFlightlessModeForHalf(NeutralSweepMode mode) {
        return mode == NeutralSweepMode.FLIGHTLESS
                || mode == NeutralSweepMode.FLIGHTLESS_WIDE
                || mode == NeutralSweepMode.FLIGHTLESS_WAVE;
    }

    private static Translation2d[] halfSweepPoints(Translation2d[] fullPoints) {
        Translation2d start = fullPoints[0];
        Translation2d end = fullPoints[fullPoints.length - 1];
        Translation2d middle = start.interpolate(end, 0.5);

        List<Translation2d> halfPoints = new ArrayList<>();
        halfPoints.add(start);
        for (int i = 1; i < fullPoints.length - 1; i++) {
            if (start.getDistance(fullPoints[i]) < start.getDistance(middle)) {
                halfPoints.add(fullPoints[i]);
            }
        }
        halfPoints.add(middle);
        return halfPoints.toArray(Translation2d[]::new);
    }

    private static Translation2d halfSweepHomeTowerPoint(Translation2d middlePoint) {
        return new Translation2d(AutoPoints.NeutralZone.LEFT_FLIGHTLESS.getX(), middlePoint.getY());
    }

    private static Rotation2d waveTangentHeading(
            Translation2d start, Translation2d end, double t, double amplitudeMeters) {
        Translation2d line = end.minus(start);
        double length = line.getNorm();
        if (length <= 1e-6) {
            return Rotation2d.kZero;
        }

        Translation2d unitLine = new Translation2d(line.getX() / length, line.getY() / length);
        Translation2d unitNormal = new Translation2d(-unitLine.getY(), unitLine.getX());
        double waveSlopeMeters = amplitudeMeters * 2.0 * Math.PI * Math.cos(2.0 * Math.PI * t);
        Translation2d tangent = line.plus(unitNormal.times(waveSlopeMeters));
        return tangent.getAngle();
    }

    private static PathPlannerPath buildWaveSweepPath(
            Translation2d leftToRightStart,
            Translation2d leftToRightEnd,
            NeutralSweepDirection direction,
            double amplitudeMeters,
            double endProgress) {
        Translation2d start = leftToRightStart;
        Translation2d end = leftToRightEnd;
        if (direction == NeutralSweepDirection.RIGHT_TO_LEFT) {
            Translation2d temp = start;
            start = end;
            end = temp;
        }

        List<Pose2d> wavePoses =
                new ArrayList<>(AutoCommandParamsNT.midWaveSampleCount.getValue().intValue());
        List<RotationTarget> rotationTargets =
                new ArrayList<>(AutoCommandParamsNT.midWaveSampleCount.getValue().intValue() - 1);
        Translation2d line = end.minus(start);
        double lineLength = line.getNorm();
        Translation2d unitNormal =
                lineLength <= 1e-6
                        ? new Translation2d(0.0, 0.0)
                        : new Translation2d(-line.getY() / lineLength, line.getX() / lineLength);
        for (int i = 0; i < AutoCommandParamsNT.midWaveSampleCount.getValue().intValue(); i++) {
            double t =
                    endProgress
                            * i
                            / (AutoCommandParamsNT.midWaveSampleCount.getValue().intValue() - 1);
            Translation2d base = start.interpolate(end, t);
            double offset = AutoPoints.NeutralZone.waveOffset(t, amplitudeMeters);
            Translation2d point = base.plus(unitNormal.times(offset));
            Rotation2d heading = waveTangentHeading(start, end, t, amplitudeMeters);
            wavePoses.add(new Pose2d(point, heading));
            if (i < AutoCommandParamsNT.midWaveSampleCount.getValue().intValue() - 1) {
                rotationTargets.add(new RotationTarget(i, heading));
            }
        }

        Pose2d startPose = wavePoses.get(0);
        Pose2d endPose = wavePoses.get(wavePoses.size() - 1);
        return new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(wavePoses),
                rotationTargets,
                List.of(),
                List.of(),
                List.of(),
                AutoParams.intakeMediumConstraints(),
                new IdealStartingState(0.0, startPose.getRotation()),
                new GoalEndState(
                        AutoCommandParamsNT.autoIntakeThroughVelocity.getValue(),
                        endPose.getRotation()),
                false);
    }

    private static Command waveNeutralZoneSweep(
            Swerve swerve,
            IntakerSubsystem intaker,
            Translation2d leftToRightStart,
            Translation2d leftToRightEnd,
            NeutralSweepDirection direction,
            double amplitudeMeters,
            MidKind kind,
            boolean skipHalfHomeLeg) {
        PathPlannerPath path =
                buildWaveSweepPath(
                        leftToRightStart,
                        leftToRightEnd,
                        direction,
                        amplitudeMeters,
                        kind == MidKind.HALF ? 0.5 : 1.0);
        capturePreviewPath(path);
        Command sweep = AutoBuilder.pathfindThenFollowPath(path, AutoParams.preciseConstraints());
        if (kind == MidKind.HALF && !skipHalfHomeLeg) {
            Pose2d endPose = path.getPathPoses().get(path.getPathPoses().size() - 1);
            sweep =
                    Commands.sequence(
                            sweep,
                            pathfindToBlueTranslationWithHeadingStrict(
                                    swerve,
                                    halfSweepHomeTowerPoint(endPose.getTranslation()),
                                    Rotation2d.kPi,
                                    AutoParams.intakeMediumConstraints(),
                                    AutoCommandParamsNT.autoIntakeThroughVelocity.getValue()));
        }
        return runWhileIntaking(sweep, intaker);
    }

    public static Command neutralZoneSweep(
            Swerve swerve,
            IntakerSubsystem intaker,
            NeutralSweepMode mode,
            NeutralSweepDirection direction) {
        return neutralZoneSweep(swerve, intaker, mode, direction, MidKind.FULL);
    }

    public static Command neutralZoneSweep(
            Swerve swerve,
            IntakerSubsystem intaker,
            NeutralSweepMode mode,
            NeutralSweepDirection direction,
            MidKind kind) {
        if (mode == NeutralSweepMode.WAVE) {
            return waveNeutralZoneSweep(
                    swerve,
                    intaker,
                    AutoPoints.NeutralZone.LEFT_CENTER,
                    AutoPoints.NeutralZone.RIGHT_CENTER,
                    direction,
                    AutoPoints.NeutralZone.WAVE_AMPLITUDE_METERS,
                    kind,
                    false);
        }
        if (mode == NeutralSweepMode.FLIGHTLESS_WAVE) {
            return waveNeutralZoneSweep(
                    swerve,
                    intaker,
                    AutoPoints.NeutralZone.LEFT_FLIGHTLESS,
                    AutoPoints.NeutralZone.RIGHT_FLIGHTLESS,
                    direction,
                    AutoPoints.NeutralZone.FLIGHTLESS_WAVE_AMPLITUDE_METERS,
                    kind,
                    true);
        }

        Rotation2d heading = neutralSweepHeading(direction);
        Translation2d[] points = neutralSweepPoints(mode, direction);
        if (kind == MidKind.HALF) {
            points = halfSweepPoints(points);
        }

        List<Command> sweepSegments = new ArrayList<>(points.length);
        for (int i = 0; i < points.length; i++) {
            sweepSegments.add(
                    pathfindToBlueTranslationWithHeadingStrict(
                            swerve,
                            points[i],
                            heading,
                            i == 0
                                    ? AutoParams.preciseConstraints()
                                    : AutoParams.intakeMediumConstraints(),
                            AutoCommandParamsNT.autoIntakeThroughVelocity.getValue()));
        }
        if (kind == MidKind.HALF && !isFlightlessModeForHalf(mode)) {
            sweepSegments.add(
                    pathfindToBlueTranslationWithHeadingStrict(
                            swerve,
                            halfSweepHomeTowerPoint(points[points.length - 1]),
                            Rotation2d.kPi,
                            AutoParams.intakeMediumConstraints(),
                            AutoCommandParamsNT.autoIntakeThroughVelocity.getValue()));
        }

        Command sweep = Commands.sequence(sweepSegments.toArray(Command[]::new));

        return runWhileIntaking(sweep, intaker);
    }

    public static Command autoAimAndShoot(
            Swerve swerve,
            IntakerSubsystem intaker,
            ShootingSuperstructure shootingSuperstructure) {
        return Commands.deadline(
                moveShotWindow(shootingSuperstructure, intaker),
                new AutoAimCommand(
                        swerve,
                        () -> 0.0,
                        () -> 0.0,
                        shootingSuperstructure::aimHeading,
                        shootingSuperstructure::aimHeadingRateRadPerSec));
    }

    public static Command driveAndShootToBlueTranslation(
            Swerve swerve,
            IntakerSubsystem intaker,
            ShootingSuperstructure shootingSuperstructure,
            Translation2d targetTranslation,
            Distance translationTolerance,
            Angle rotationTolerance) {
        capturePreviewTarget(new Pose2d(targetTranslation, Rotation2d.kZero));
        return Commands.parallel(
                driveToDynamicPose(
                                swerve,
                                () ->
                                        new Pose2d(
                                                AllianceFlipUtil.apply(targetTranslation),
                                                shootingSuperstructure.aimHeading()),
                                new PIDController(
                                        AutoCommandParamsNT.moveShotTranslationKP.getValue(),
                                        0.0,
                                        AutoCommandParamsNT.moveShotTranslationKD.getValue()),
                                new PIDController(
                                        AutoCommandParamsNT.moveShotRotationKP.getValue(),
                                        0.0,
                                        AutoCommandParamsNT.moveShotRotationKD.getValue()),
                                translationTolerance,
                                rotationTolerance)
                        .withTimeout(AutoCommandParamsNT.autoReturnTimeoutSeconds.getValue()),
                moveShotWindowShort(shootingSuperstructure, intaker));
    }

    private static Pose2d depotStartPose(AutoSelector.DepotAxis depotAxis) {
        return depotAxis == AutoSelector.DepotAxis.X
                ? AutoPoints.DEPOT_X_START
                : AutoPoints.DEPOT_Y_START;
    }

    private static Pose2d depotEndPose(AutoSelector.DepotAxis depotAxis) {
        return depotAxis == AutoSelector.DepotAxis.X
                ? AutoPoints.DEPOT_X_END
                : AutoPoints.DEPOT_Y_END;
    }

    private static Command depotCollectFromSelection(
            Swerve swerve, IntakerSubsystem intaker, AutoSelector.DepotAxis depotAxis) {
        return depotAxis == AutoSelector.DepotAxis.X
                ? depotXCollect(swerve, intaker)
                : depotYCollect(swerve, intaker);
    }

    private static Command depotCollectFromCurrentStartSelection(
            Swerve swerve, IntakerSubsystem intaker, AutoSelector.DepotAxis depotAxis) {
        return runWhileIntaking(
                pathfindToBluePoseStrict(
                        swerve, depotEndPose(depotAxis), AutoParams.intakeMediumConstraints(), 0.0),
                intaker);
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
                        intaker,
                        shootingSuperstructure,
                        depotStart.getTranslation(),
                        Units.Meters.of(0.15),
                        Units.Degrees.of(5.0)),
                markStep(stepPrefix + ": depot collect"),
                depotCollectFromCurrentStartSelection(swerve, intaker, depotAxis));
    }

    private static Command moveShotToOutpost(
            Swerve swerve,
            IntakerSubsystem intaker,
            ShootingSuperstructure shootingSuperstructure,
            String stepPrefix) {
        return Commands.sequence(
                markStep(stepPrefix + ": outpost approach"),
                pathfindToBluePoseStrict(
                                swerve,
                                AutoPoints.OUTPOST_APPROACH,
                                AutoParams.transitConstraints(),
                                AutoCommandParamsNT.autoThroughVelocity.getValue())
                        .withTimeout(AutoCommandParamsNT.autoReturnTimeoutSeconds.getValue()),
                markStep(stepPrefix + ": move-shot to outpost"),
                driveAndShootToBlueTranslation(
                        swerve,
                        intaker,
                        shootingSuperstructure,
                        AutoPoints.OUTPOST.getTranslation(),
                        Units.Meters.of(0.15),
                        Units.Degrees.of(5.0)));
    }

    private static Command handlePostSweepAction(
            Swerve swerve,
            IntakerSubsystem intaker,
            ShootingSuperstructure shootingSuperstructure,
            AutoSelector.Side shootPosition,
            AutoSelector.DepotAxis depotAxis,
            DepotVisitRound depotRound,
            DepotVisitRound outpostRound,
            int roundIndex) {
        DepotVisitRound currentRound =
                roundIndex == 1 ? DepotVisitRound.FIRST : DepotVisitRound.SECOND;

        if (depotRound == currentRound) {
            return Commands.sequence(
                    markStep("Round " + roundIndex + ": launch pose"),
                    goToBumpLaunchForSweepEnd(
                                    swerve,
                                    shootPosition,
                                    AutoCommandParamsNT.autoThroughVelocity.getValue())
                            .withTimeout(AutoCommandParamsNT.autoReturnTimeoutSeconds.getValue()),
                    shootThenDepotCollect(
                            swerve,
                            intaker,
                            shootingSuperstructure,
                            depotAxis,
                            "Round " + roundIndex));
        }

        if (outpostRound == currentRound) {
            return moveShotToOutpost(
                    swerve, intaker, shootingSuperstructure, "Round " + roundIndex);
        }

        return Commands.sequence(
                markStep("Round " + roundIndex + ": launch pose"),
                goToBumpLaunchForSweepEnd(swerve, shootPosition)
                        .withTimeout(AutoCommandParamsNT.autoReturnTimeoutSeconds.getValue()),
                markStep("Round " + roundIndex + ": shoot"),
                autoAimAndShoot(swerve, intaker, shootingSuperstructure));
    }

    public static Command goToBumpLaunchForSweepEnd(
            Swerve swerve, NeutralSweepDirection direction) {
        return goToBumpLaunchForSweepEnd(swerve, direction, 0.0);
    }

    public static Command goToBumpLaunchForSweepEnd(
            Swerve swerve, AutoSelector.Side shootPosition) {
        return goToBumpLaunchForSweepEnd(swerve, shootPosition, 0.0);
    }

    private static Command goToBumpLaunchForSweepEnd(
            Swerve swerve, NeutralSweepDirection direction, double goalEndVelocityMetersPerSecond) {
        return direction == NeutralSweepDirection.LEFT_TO_RIGHT
                ? goToBumpLaunchForSweepEnd(
                        swerve, AutoSelector.Side.RIGHT, goalEndVelocityMetersPerSecond)
                : goToBumpLaunchForSweepEnd(
                        swerve, AutoSelector.Side.LEFT, goalEndVelocityMetersPerSecond);
    }

    private static Command goToBumpLaunchForSweepEnd(
            Swerve swerve, AutoSelector.Side shootPosition, double goalEndVelocityMetersPerSecond) {
        return shootPosition == AutoSelector.Side.LEFT
                ? pathfindToBlueTranslationPreserveHeading(
                        AutoPoints.Launch.LEFT_BUMP.getTranslation(),
                        AutoParams.transitConstraints(),
                        goalEndVelocityMetersPerSecond)
                : pathfindToBlueTranslationPreserveHeading(
                        AutoPoints.Launch.RIGHT_BUMP.getTranslation(),
                        AutoParams.transitConstraints(),
                        goalEndVelocityMetersPerSecond);
    }

    public static Command midTwoCycle(
            Swerve swerve,
            IntakerSubsystem intaker,
            ShootingSuperstructure shootingSuperstructure,
            NeutralSweepMode firstMode,
            NeutralSweepMode secondMode,
            NeutralSweepDirection firstDirection,
            NeutralSweepDirection secondDirection,
            MidKind firstKind,
            MidKind secondKind,
            AutoSelector.Side firstShootPosition,
            AutoSelector.Side secondShootPosition,
            AutoSelector.DepotAxis depotAxis,
            DepotVisitRound depotRound,
            DepotVisitRound outpostRound) {
        Command startAction =
                depotRound == DepotVisitRound.START
                        ? Commands.sequence(
                                markStep("Start: depot collect"),
                                depotCollectFromSelection(swerve, intaker, depotAxis))
                        : Commands.none();
        return Commands.sequence(
                        startAction,
                        markStep("Mid Two Cycle: first intake"),
                        neutralZoneSweep(swerve, intaker, firstMode, firstDirection, firstKind),
                        handlePostSweepAction(
                                swerve,
                                intaker,
                                shootingSuperstructure,
                                firstShootPosition,
                                depotAxis,
                                depotRound,
                                outpostRound,
                                1),
                        markStep("Mid Two Cycle: second intake"),
                        neutralZoneSweep(swerve, intaker, secondMode, secondDirection, secondKind),
                        handlePostSweepAction(
                                swerve,
                                intaker,
                                shootingSuperstructure,
                                secondShootPosition,
                                depotAxis,
                                depotRound,
                                outpostRound,
                                2))
                .withTimeout(AutoCommandParamsNT.autoDurationSeconds.getValue());
    }

    private static Command depotCollect(
            Swerve swerve, IntakerSubsystem intaker, Pose2d startPose, Pose2d endPose) {
        return Commands.sequence(
                pathfindToBluePoseStrict(
                        swerve,
                        startPose,
                        AutoParams.preciseConstraints(),
                        AutoCommandParamsNT.autoIntakeThroughVelocity.getValue()),
                runWhileIntaking(
                        pathfindToBluePoseStrict(
                                swerve, endPose, AutoParams.intakeMediumConstraints(), 0.0),
                        intaker));
    }

    /** DEPOT X START -> intake on -> DEPOT X END */
    public static Command depotXCollect(Swerve swerve, IntakerSubsystem intaker) {
        return depotCollect(swerve, intaker, AutoPoints.DEPOT_X_START, AutoPoints.DEPOT_X_END);
    }

    /** DEPOT Y START -> intake on -> DEPOT Y END */
    public static Command depotYCollect(Swerve swerve, IntakerSubsystem intaker) {
        return depotCollect(swerve, intaker, AutoPoints.DEPOT_Y_START, AutoPoints.DEPOT_Y_END);
    }

    /** Move to the OUTPOST pose. */
    public static Command goToOutpost(Swerve swerve) {
        return pathfindToBluePoseStrict(
                swerve, AutoPoints.OUTPOST, AutoParams.transitConstraints(), 0.0);
    }

    public static Command goToHubCenterStart(Swerve swerve) {
        return pathfindToBlueTranslationWithHeadingStrict(
                swerve,
                AutoPoints.Hub.CENTER_START,
                Rotation2d.kPi,
                AutoParams.transitConstraints(),
                0.0);
    }

    public static Command trenchLeftStartToClear(Swerve swerve) {
        return Commands.sequence(
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.Trench.LEFT_START,
                        Rotation2d.kPi,
                        AutoParams.preciseConstraints(),
                        AutoCommandParamsNT.autoThroughVelocity.getValue()),
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.Trench.LEFT_CLEAR,
                        Rotation2d.kPi,
                        AutoParams.transitConstraints(),
                        0.0));
    }

    public static Command trenchRightStartToClear(Swerve swerve) {
        return Commands.sequence(
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.Trench.RIGHT_START,
                        Rotation2d.kPi,
                        AutoParams.preciseConstraints(),
                        AutoCommandParamsNT.autoThroughVelocity.getValue()),
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.Trench.RIGHT_CLEAR,
                        Rotation2d.kPi,
                        AutoParams.transitConstraints(),
                        0.0));
    }

    public static Command bumpLeftInnerToOuter(Swerve swerve) {
        return Commands.sequence(
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.Bump.LEFT_INNER,
                        Rotation2d.kPi,
                        AutoParams.preciseConstraints(),
                        AutoCommandParamsNT.autoThroughVelocity.getValue()),
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.Bump.LEFT_OUTER,
                        Rotation2d.kPi,
                        AutoParams.transitConstraints(),
                        0.0));
    }

    public static Command bumpRightInnerToOuter(Swerve swerve) {
        return Commands.sequence(
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.Bump.RIGHT_INNER,
                        Rotation2d.kPi,
                        AutoParams.preciseConstraints(),
                        AutoCommandParamsNT.autoThroughVelocity.getValue()),
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.Bump.RIGHT_OUTER,
                        Rotation2d.kPi,
                        AutoParams.transitConstraints(),
                        0.0));
    }

    public static Command depotLeftThrough(Swerve swerve, IntakerSubsystem intaker) {
        return runWhileIntaking(
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.Depot.LEFT_THROUGH,
                        Rotation2d.kZero,
                        AutoParams.intakeMediumConstraints(),
                        0.0),
                intaker);
    }

    public static Command depotRightThrough(Swerve swerve, IntakerSubsystem intaker) {
        return runWhileIntaking(
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.Depot.RIGHT_THROUGH,
                        Rotation2d.kZero,
                        AutoParams.intakeMediumConstraints(),
                        0.0),
                intaker);
    }

    public static Command towerLeftThrough(Swerve swerve) {
        return pathfindToBlueTranslationWithHeadingStrict(
                swerve,
                AutoPoints.Tower.LEFT_THROUGH,
                Rotation2d.kZero,
                AutoParams.preciseConstraints(),
                0.0);
    }

    public static Command towerRightThrough(Swerve swerve) {
        return pathfindToBlueTranslationWithHeadingStrict(
                swerve,
                AutoPoints.Tower.RIGHT_THROUGH,
                Rotation2d.kZero,
                AutoParams.preciseConstraints(),
                0.0);
    }

    public static Command goToLeftBumpLaunch(Swerve swerve) {
        return pathfindToBluePoseStrict(
                swerve, AutoPoints.Launch.LEFT_BUMP, AutoParams.transitConstraints(), 0.0);
    }

    public static Command goToRightBumpLaunch(Swerve swerve) {
        return pathfindToBluePoseStrict(
                swerve, AutoPoints.Launch.RIGHT_BUMP, AutoParams.transitConstraints(), 0.0);
    }

    public static Command goToLeftTrenchLaunch(Swerve swerve) {
        return pathfindToBluePoseStrict(
                swerve, AutoPoints.Launch.LEFT_TRENCH, AutoParams.transitConstraints(), 0.0);
    }

    public static Command goToRightTrenchLaunch(Swerve swerve) {
        return pathfindToBluePoseStrict(
                swerve, AutoPoints.Launch.RIGHT_TRENCH, AutoParams.transitConstraints(), 0.0);
    }

    public static Command goToLeftClimb(Swerve swerve) {
        return pathfindToBluePoseStrict(
                swerve, AutoPoints.Climb.LEFT, AutoParams.preciseConstraints(), 0.0);
    }

    public static Command goToRightClimb(Swerve swerve) {
        return pathfindToBluePoseStrict(
                swerve, AutoPoints.Climb.RIGHT, AutoParams.preciseConstraints(), 0.0);
    }

    /** Sweep the middle line from left to right while facing -90 degrees and intaking. */
    public static Command sweepMidLeftToRight(Swerve swerve, IntakerSubsystem intaker) {
        return neutralZoneSweep(
                swerve, intaker, NeutralSweepMode.SALESMAN, NeutralSweepDirection.LEFT_TO_RIGHT);
    }

    /** Sweep the middle line from right to left while facing 90 degrees and intaking. */
    public static Command sweepMidRightToLeft(Swerve swerve, IntakerSubsystem intaker) {
        return neutralZoneSweep(
                swerve, intaker, NeutralSweepMode.SALESMAN, NeutralSweepDirection.RIGHT_TO_LEFT);
    }
}
