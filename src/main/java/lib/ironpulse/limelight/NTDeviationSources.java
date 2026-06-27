package lib.ironpulse.limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A {@link DeviationParamSources} implementation backed by NetworkTables, so vision
 * standard-deviation parameters (xStdDev, yStdDev, zStdDev, angleStdDev) and the IMU correction
 * reliability threshold can be tuned live through Shuffleboard / Glass without re-deploying code.
 *
 * <p>Each limelight gets its own namespace: {@code Limelight/<name>/debug/<param>}.
 */
public class NTDeviationSources implements DeviationParamSources {

    private final String name;

    // Hard-coded fallback defaults that also seed the NT entries on construction.
    private static final double DEFAULT_X_STD_DEV = 0.7;
    private static final double DEFAULT_Y_STD_DEV = 0.7;
    private static final double DEFAULT_Z_STD_DEV = 9999.0;
    private static final double DEFAULT_ANGLE_STD_DEV = 1.0;
    private static final double DEFAULT_IMU_THRESHOLD = 0.9;

    /**
     * @param name the limelight name (e.g. "limelight"), used for the NT key prefix
     */
    public NTDeviationSources(String name) {
        this.name = name;
        // Publish defaults so the entries appear in Shuffleboard / Glass for live tuning.
        SmartDashboard.putNumber(key("xStdDev"), DEFAULT_X_STD_DEV);
        SmartDashboard.putNumber(key("yStdDev"), DEFAULT_Y_STD_DEV);
        SmartDashboard.putNumber(key("zStdDev"), DEFAULT_Z_STD_DEV);
        SmartDashboard.putNumber(key("angleStdDev"), DEFAULT_ANGLE_STD_DEV);
        SmartDashboard.putNumber(key("imuCorrectionReliabilityThreshold"), DEFAULT_IMU_THRESHOLD);
    }

    private String key(String param) {
        return "Limelight/" + name + "/debug/" + param;
    }

    @Override
    public double xStdDev() {
        return SmartDashboard.getNumber(key("xStdDev"), DEFAULT_X_STD_DEV);
    }

    @Override
    public double yStdDev() {
        return SmartDashboard.getNumber(key("yStdDev"), DEFAULT_Y_STD_DEV);
    }

    @Override
    public double zStdDev() {
        return SmartDashboard.getNumber(key("zStdDev"), DEFAULT_Z_STD_DEV);
    }

    @Override
    public double angleStdDev() {
        return SmartDashboard.getNumber(key("angleStdDev"), DEFAULT_ANGLE_STD_DEV);
    }

    @Override
    public double imuCorrectionReliabilityThreshold() {
        return SmartDashboard.getNumber(key("imuCorrectionReliabilityThreshold"), DEFAULT_IMU_THRESHOLD);
    }
}
