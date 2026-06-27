package lib.ironpulse.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * A {@link DeviationParamSources} implementation backed by NetworkTables, so vision
 * standard-deviation parameters (xStdDev, yStdDev, zStdDev, angleStdDev) and the IMU correction
 * reliability threshold can be tuned live through Glass / Shuffleboard without re-deploying code.
 *
 * <p>Entries are published under {@code /LimelightDebug/<name>/} (a top-level table in Glass).
 * Each entry is seeded with a default value via {@code setDefault()} so it appears immediately
 * and is never overwritten once the user has set a custom value.
 */
public class NTDeviationSources implements DeviationParamSources {

    private final NetworkTable table;
    private final double defaultXStdDev;
    private final double defaultYStdDev;
    private final double defaultZStdDev;
    private final double defaultAngleStdDev;
    private final double defaultImuThreshold;

    /**
     * @param name                    the limelight name (e.g. "limelight"), used as a subtable
     * @param defaultXStdDev          default x standard deviation
     * @param defaultYStdDev          default y standard deviation
     * @param defaultZStdDev          default z standard deviation
     * @param defaultAngleStdDev      default angle standard deviation
     * @param defaultImuThreshold     default IMU correction reliability threshold
     */
    public NTDeviationSources(
            String name,
            double defaultXStdDev,
            double defaultYStdDev,
            double defaultZStdDev,
            double defaultAngleStdDev,
            double defaultImuThreshold) {
        this.table = NetworkTableInstance.getDefault()
                .getTable("LimelightDebug")
                .getSubTable(name);
        this.defaultXStdDev = defaultXStdDev;
        this.defaultYStdDev = defaultYStdDev;
        this.defaultZStdDev = defaultZStdDev;
        this.defaultAngleStdDev = defaultAngleStdDev;
        this.defaultImuThreshold = defaultImuThreshold;

        // setDefaultValue creates the NT topic with a fallback value but never overwrites
        // a value the user has already set through Glass / Shuffleboard.
        table.getEntry("xStdDev").setDefaultValue(defaultXStdDev);
        table.getEntry("yStdDev").setDefaultValue(defaultYStdDev);
        table.getEntry("zStdDev").setDefaultValue(defaultZStdDev);
        table.getEntry("angleStdDev").setDefaultValue(defaultAngleStdDev);
        table.getEntry("imuCorrectionReliabilityThreshold").setDefaultValue(defaultImuThreshold);
    }

    @Override
    public double xStdDev() {
        return table.getEntry("xStdDev").getDouble(defaultXStdDev);
    }

    @Override
    public double yStdDev() {
        return table.getEntry("yStdDev").getDouble(defaultYStdDev);
    }

    @Override
    public double zStdDev() {
        return table.getEntry("zStdDev").getDouble(defaultZStdDev);
    }

    @Override
    public double angleStdDev() {
        return table.getEntry("angleStdDev").getDouble(defaultAngleStdDev);
    }

    @Override
    public double imuCorrectionReliabilityThreshold() {
        return table.getEntry("imuCorrectionReliabilityThreshold").getDouble(defaultImuThreshold);
    }
}
