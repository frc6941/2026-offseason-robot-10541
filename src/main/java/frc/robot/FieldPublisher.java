package frc.robot;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Publishes a {@link Field2d} (NT key "Field") for Elastic's Field widget: the robot pose plus the
 * active PathPlanner path / target pose.
 *
 * <p>Lives in {@code frc.robot} (not the vendored lib) so future lib copies don't wipe it.
 * Supersedes the unused {@code lib.ironpulse.display.FieldView}.
 */
public final class FieldPublisher {
    private static final Field2d field = new Field2d();
    private static boolean initialized = false;

    private FieldPublisher() {}

    /**
     * Publish the Field2d and hook PathPlanner's active-path/target logging. Call once at startup,
     * before any path runs (the callbacks are global and fire whenever PathPlanner follows/pathfinds).
     */
    public static void init() {
        if (initialized) {
            return;
        }
        SmartDashboard.putData("Field", field);
        PathPlannerLogging.setLogActivePathCallback(
                poses -> field.getObject("AutoPath").setPoses(poses));
        PathPlannerLogging.setLogTargetPoseCallback(
                pose -> field.getObject("AutoTarget").setPose(pose));
        initialized = true;
    }

    /** Update the robot's pose on the field. Call every loop. */
    public static void setRobotPose(Pose2d pose) {
        field.setRobotPose(pose);
    }

    /** Draw the selected auto's target waypoints (preview, before the auto runs). */
    public static void setPreview(java.util.List<Pose2d> poses) {
        field.getObject("AutoPreview").setPoses(poses);
    }
}
