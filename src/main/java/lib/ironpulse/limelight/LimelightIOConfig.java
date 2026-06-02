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

    private final MountPosition mountPosition;

    private final Limelight4Config limeLight4Config;

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
