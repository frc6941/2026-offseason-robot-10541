package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intaker.IntakerSubsystem;

public final class AutoCommands {
    public static final PathConstraints PRECISE_CONSTRAINTS =
            new PathConstraints(1.5, 2.0, Math.toRadians(360.0), Math.toRadians(540.0));
    public static final PathConstraints INTAKE_MEDIUM_CONSTRAINTS =
            new PathConstraints(2.0, 2.5, Math.toRadians(360.0), Math.toRadians(540.0));
    public static final PathConstraints TRANSIT_CONSTRAINTS =
            new PathConstraints(3.0, 4.0, Math.toRadians(540.0), Math.toRadians(720.0));

    private AutoCommands() {}

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
    public static Command depotXCollect(IntakerSubsystem intaker) {
        return Commands.sequence(
                pathfindToBluePose(AutoPoints.DEPOT_X_START, PRECISE_CONSTRAINTS, 0.0),
                intaker.runIntake(),
                pathfindToBluePose(AutoPoints.DEPOT_X_END, INTAKE_MEDIUM_CONSTRAINTS, 0.0));
    }

    /** DEPOT Y START -> intake on -> DEPOT Y END */
    public static Command depotYCollect(IntakerSubsystem intaker) {
        return Commands.sequence(
                pathfindToBluePose(AutoPoints.DEPOT_Y_START, PRECISE_CONSTRAINTS, 0.0),
                intaker.runIntake(),
                pathfindToBluePose(AutoPoints.DEPOT_Y_END, INTAKE_MEDIUM_CONSTRAINTS, 0.0));
    }

    /** Move to the OUTPOST pose. */
    public static Command goToOutpost() {
        return pathfindToBluePose(AutoPoints.OUTPOST, TRANSIT_CONSTRAINTS, 0.0);
    }

    /** Sweep the middle line from left to right while facing -90 degrees and intaking. */
    public static Command sweepMidLeftToRight(IntakerSubsystem intaker) {
        return Commands.sequence(
                pathfindToBlueTranslationWithHeading(
                        AutoPoints.MID_LEFT,
                        Rotation2d.fromDegrees(-90.0),
                        INTAKE_MEDIUM_CONSTRAINTS,
                        0.0),
                intaker.runIntake(),
                pathfindToBlueTranslationWithHeading(
                        AutoPoints.MID_RIGHT,
                        Rotation2d.fromDegrees(-90.0),
                        INTAKE_MEDIUM_CONSTRAINTS,
                        0.0));
    }

    /** Sweep the middle line from right to left while facing 90 degrees and intaking. */
    public static Command sweepMidRightToLeft(IntakerSubsystem intaker) {
        return Commands.sequence(
                pathfindToBlueTranslationWithHeading(
                        AutoPoints.MID_RIGHT,
                        Rotation2d.fromDegrees(90.0),
                        INTAKE_MEDIUM_CONSTRAINTS,
                        0.0),
                intaker.runIntake(),
                pathfindToBlueTranslationWithHeading(
                        AutoPoints.MID_LEFT,
                        Rotation2d.fromDegrees(90.0),
                        INTAKE_MEDIUM_CONSTRAINTS,
                        0.0));
    }
}
