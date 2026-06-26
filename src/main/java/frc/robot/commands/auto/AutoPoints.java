package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Blue-alliance field reference points for pathfinding autos.
 *
 * <p>These poses are written in the global blue-origin field frame. Commands should use the flipped
 * PathPlanner pathfind APIs so the same definitions work on both alliances.
 */
public final class AutoPoints {
    public static final Pose2d DEPOT_X_START =
            new Pose2d(1.5, 5.95, Rotation2d.fromDegrees(180.0));
    public static final Pose2d DEPOT_X_END =
            new Pose2d(0.491, 5.95, Rotation2d.fromDegrees(180.0));

    public static final Pose2d DEPOT_Y_START =
            new Pose2d(0.491, 7.3, Rotation2d.fromDegrees(-90.0));
    public static final Pose2d DEPOT_Y_END =
            new Pose2d(0.491, 5.95, Rotation2d.fromDegrees(-90.0));

    public static final Pose2d START_LEFT =
            new Pose2d(2.0, 5.95, Rotation2d.fromDegrees(0.0));
    public static final Pose2d START_RIGHT =
            new Pose2d(2.0, 2.15, Rotation2d.fromDegrees(0.0));

    public static final Translation2d MID_LEFT = new Translation2d(8.25, 7.2);
    public static final Translation2d MID_RIGHT = new Translation2d(8.25, 0.8);

    public static final Pose2d OUTPOST =
            new Pose2d(0.491, 0.649, Rotation2d.fromDegrees(0.0));

    private AutoPoints() {}
}
