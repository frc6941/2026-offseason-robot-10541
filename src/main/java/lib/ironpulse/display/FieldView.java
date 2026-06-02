package lib.ironpulse.display;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldView {
    private static final Field2d mField2d = new Field2d();

    public static void Init() {
        SmartDashboard.putData(mField2d);
    }

    public static void updateObjectPose(Pose2d pose, String name) {
        mField2d.getObject(name).setPose(pose);
    }

    public static void updateObjectTraj(Trajectory traj, String name) {
        mField2d.getObject(name).setTrajectory(traj);
    }

    public static void removeObject(String name) {
        mField2d.getObject(name).close();
    }

    public static void updateRobotPose(Pose2d pose) {
        mField2d.setRobotPose(pose);
    }
}
