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
    private final double defaultXStdDev;
    private final double defaultYStdDev;
    private final double defaultZStdDev;
    private final double defaultAngleStdDev;
    private final double defaultImuThreshold;

    /**
     * @param name              the limelight name (e.g. "limelight"), used for the NT key prefix
     * @param defaultXStdDev    default x standard deviation
     * @param defaultYStdDev    default y standard deviation
     * @param defaultZStdDev    default z standard deviation
     * @param defaultAngleStdDev default angle standard deviation
     * @param defaultImuThreshold default IMU correction reliability threshold
     */
    public NTDeviationSources(
            String name,
            double defaultXStdDev,
            double defaultYStdDev,
            double defaultZStdDev,
            double defaultAngleStdDev,
            double defaultImuThreshold) {
        this.name = name;
        this.defaultXStdDev = defaultXStdDev;
        this.defaultYStdDev = defaultYStdDev;
        this.defaultZStdDev = defaultZStdDev;
        this.defaultAngleStdDev = defaultAngleStdDev;
        this.defaultImuThreshold = defaultImuThreshold;
    }

    private String key(String param) {
        return "Limelight/" + name + "/debug/" + param;
    }

    @Override
    public double xStdDev() {
        double value = SmartDashboard.getNumber(key("xStdDev"), defaultXStdDev);
        SmartDashboard.putNumber(key("xStdDev"), value);
        return value;
    }

    @Override
    public double yStdDev() {
        double value = SmartDashboard.getNumber(key("yStdDev"), defaultYStdDev);
        SmartDashboard.putNumber(key("yStdDev"), value);
        return value;
    }

    @Override
    public double zStdDev() {
        double value = SmartDashboard.getNumber(key("zStdDev"), defaultZStdDev);
        SmartDashboard.putNumber(key("zStdDev"), value);
        return value;
    }

    @Override
    public double angleStdDev() {
        double value = SmartDashboard.getNumber(key("angleStdDev"), defaultAngleStdDev);
        SmartDashboard.putNumber(key("angleStdDev"), value);
        return value;
    }

    @Override
    public double imuCorrectionReliabilityThreshold() {
        double value = SmartDashboard.getNumber(key("imuCorrectionReliabilityThreshold"), defaultImuThreshold);
        SmartDashboard.putNumber(key("imuCorrectionReliabilityThreshold"), value);
        return value;
    }
}
