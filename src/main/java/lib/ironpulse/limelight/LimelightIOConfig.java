package lib.ironpulse.limelight;

import lombok.Builder;
import lombok.Getter;

@Getter
@Builder
public class LimelightIOConfig {
    @Builder.Default private final boolean useMegaTag2 = true;
    // weight given to its estimation
    // ranges from 0-1 inclusive.
    @Builder.Default private final double weight = 1.0;
    // complementary filter alpha for the internal IMU. Default is 0.001.
    // Lower values (e.g., 0.001): Smoother, slower drift correction. The internal IMU is trusted
    // more.
    // Higher values (e.g., 0.01): Faster tracking of the reference source (MT1 or external IMU).
    @Builder.Default private final double filterAlpha = 0.001;
    @Builder.Default private final String name = "UNNAMED";
    @Builder.Default private final int portToForwardStream = 0;
    @Builder.Default private final int portToForwardPipeline = 0;

    /**
     * When enabled, standard-deviation parameters (xStdDev, yStdDev, zStdDev, angleStdDev,
     * imuCorrectionReliabilityThreshold) are read from NetworkTables at runtime so they can be
     * tuned live via Shuffleboard / Glass without re-deploying code.
     *
     * @see #createDeviationSources()
     */
    @Builder.Default private final boolean debug = false;

    private final MountPosition mountPosition;

    private final Limelight4Config limeLight4Config;

    /**
     * Creates a {@link DeviationParamSources} instance. When {@link #debug} is true, returns an
     * NT-tunable implementation that reads/writes values via SmartDashboard so they can be adjusted
     * live. When {@link #debug} is false, returns {@code null} — the caller should fall back to
     * hard-coded defaults.
     *
     * @return a tunable {@code DeviationParamSources} if debug is enabled, or {@code null}
     */
    public DeviationParamSources createDeviationSources() {
        if (debug) {
            return new NTDeviationSources(name);
        }
        return null;
    }

    public enum MountPosition {
        ON_ROBOT,
        ON_MECHANISM
    }

    @Getter
    @Builder
    public static class Limelight4Config {
        @Builder.Default private final boolean useInternalIMU = true;
        @Builder.Default private final int throttleWhenDisabled = 200;
        @Builder.Default private final int throttleWhenEnabled = 0;
    }
}
