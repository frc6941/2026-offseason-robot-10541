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

    // --- vision standard-deviation defaults ---
    // These seed the NT entries and also act as fallback values when a Shuffleboard slider has
    // not been touched yet. They can be set directly in the builder or tuned live via NT.
    @Builder.Default private final double defaultXStdDev = 0.7;
    @Builder.Default private final double defaultYStdDev = 0.7;
    @Builder.Default private final double defaultZStdDev = 9999.0;
    @Builder.Default private final double defaultAngleStdDev = 1.0;
    @Builder.Default private final double defaultImuCorrectionReliabilityThreshold = 0.9;

    private final MountPosition mountPosition;

    private final Limelight4Config limeLight4Config;

    /**
     * Creates a {@link DeviationParamSources} backed by NetworkTables so standard-deviation
     * parameters can be tuned live through Shuffleboard / Glass without re-deploying code. The
     * default values set in this config seed the NT entries and act as fallbacks when a slider
     * has not been touched yet.
     *
     * <p>NT keys: {@code Limelight/<name>/debug/<param>}
     *
     * @return a tunable {@code DeviationParamSources} (never null)
     */
    public DeviationParamSources createDeviationSources() {
        return new NTDeviationSources(
                name,
                defaultXStdDev,
                defaultYStdDev,
                defaultZStdDev,
                defaultAngleStdDev,
                defaultImuCorrectionReliabilityThreshold);
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
