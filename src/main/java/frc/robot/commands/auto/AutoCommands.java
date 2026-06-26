package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.commands.SwerveAimToHeading;
import lib.ironpulse.utils.AllianceFlipUtil;

public final class AutoCommands {
    public static final PathConstraints PRECISE_CONSTRAINTS =
            new PathConstraints(1.5, 2.0, Math.toRadians(360.0), Math.toRadians(540.0));
    public static final PathConstraints INTAKE_MEDIUM_CONSTRAINTS =
            new PathConstraints(2.0, 2.5, Math.toRadians(360.0), Math.toRadians(540.0));
    public static final PathConstraints TRANSIT_CONSTRAINTS =
            new PathConstraints(3.0, 4.0, Math.toRadians(540.0), Math.toRadians(720.0));
    private static final double AUTO_TRANSLATION_TOLERANCE_METERS = 0.10;
    private static final Angle AUTO_ROTATION_TOLERANCE = Units.Degrees.of(3.0);

    private AutoCommands() {}

    private static Rotation2d flipBlueHeading(Rotation2d blueHeading) {
        return AllianceFlipUtil.apply(blueHeading);
    }

    private static Pose2d flipBluePose(Pose2d bluePose) {
        return AllianceFlipUtil.apply(bluePose);
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
    public static Command pathfindToPose(Pose2d targetPose, PathConstraints constraints) {
        return AutoBuilder.pathfindToPose(targetPose, constraints);
    }

    /**
     * Pathfind to a blue-alliance pose and let PathPlanner flip it automatically for the current
     * alliance.
     */
    public static Command pathfindToBluePose(Pose2d targetPose, PathConstraints constraints) {
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

    /** DEPOT X START -> intake on -> DEPOT X END */
    public static Command depotXCollect(Swerve swerve, IntakerSubsystem intaker) {
        return Commands.sequence(
                pathfindToBluePoseStrict(swerve, AutoPoints.DEPOT_X_START, PRECISE_CONSTRAINTS, 0.0),
                intaker.runIntake(),
                pathfindToBluePoseStrict(swerve, AutoPoints.DEPOT_X_END, INTAKE_MEDIUM_CONSTRAINTS, 0.0));
    }

    /** DEPOT Y START -> intake on -> DEPOT Y END */
    public static Command depotYCollect(Swerve swerve, IntakerSubsystem intaker) {
        return Commands.sequence(
                pathfindToBluePoseStrict(swerve, AutoPoints.DEPOT_Y_START, PRECISE_CONSTRAINTS, 0.0),
                intaker.runIntake(),
                pathfindToBluePoseStrict(swerve, AutoPoints.DEPOT_Y_END, INTAKE_MEDIUM_CONSTRAINTS, 0.0));
    }

    /** Move to the OUTPOST pose. */
    public static Command goToOutpost(Swerve swerve) {
        return pathfindToBluePoseStrict(swerve, AutoPoints.OUTPOST, TRANSIT_CONSTRAINTS, 0.0);
    }

    /** Sweep the middle line from left to right while facing -90 degrees and intaking. */
    public static Command sweepMidLeftToRight(Swerve swerve, IntakerSubsystem intaker) {
        return Commands.sequence(
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.MID_LEFT,
                        Rotation2d.fromDegrees(-90.0),
                        INTAKE_MEDIUM_CONSTRAINTS,
                        0.0),
                intaker.runIntake(),
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.MID_RIGHT,
                        Rotation2d.fromDegrees(-90.0),
                        INTAKE_MEDIUM_CONSTRAINTS,
                        0.0));
    }

    /** Sweep the middle line from right to left while facing 90 degrees and intaking. */
    public static Command sweepMidRightToLeft(Swerve swerve, IntakerSubsystem intaker) {
        return Commands.sequence(
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.MID_RIGHT,
                        Rotation2d.fromDegrees(90.0),
                        INTAKE_MEDIUM_CONSTRAINTS,
                        0.0),
                intaker.runIntake(),
                pathfindToBlueTranslationWithHeadingStrict(
                        swerve,
                        AutoPoints.MID_LEFT,
                        Rotation2d.fromDegrees(90.0),
                        INTAKE_MEDIUM_CONSTRAINTS,
                        0.0));
    }
}
