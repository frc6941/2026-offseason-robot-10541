package lib.ironpulse.limelight;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;

// any swerve that implements these methods can be provided to LimelightSubsystem.
public interface Localizable {
    void addVisionMeasurement(
            Pose3d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N4, N1> visionMeasurementStdDevs);

    double getIMUYaw();

    void setIMUYaw(double yaw);
}
