package lib.ironpulse.limelight;

// TODO: dynamic deviation,
// see: https://www.chiefdelphi.com/t/questions-on-how-to-tune-vision-standard-deviations/509750
/** Runtime tuning sources for limelight vision covariance and IMU correction threshold. */
public interface DeviationParamSources {
    double xStdDev();

    double yStdDev();

    double zStdDev();

    double angleStdDev();

    double imuCorrectionReliabilityThreshold();
}
