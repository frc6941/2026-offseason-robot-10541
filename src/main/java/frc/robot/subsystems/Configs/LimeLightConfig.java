package frc.robot.subsystems.Configs;

import lib.ironpulse.limelight.DeviationParamSources;
import lib.ironpulse.limelight.LimelightIOConfig;
import lib.ntext.NTParameter;

public class LimeLightConfig {
    public static final String NAME = "limelight-a";
    public static final LimelightIOConfig limelightConfig =
            LimelightIOConfig.builder()
                    .name(NAME)
                    .useMegaTag2(true)
                    .mountPosition(LimelightIOConfig.MountPosition.ON_ROBOT)
                    .portToForwardStream(5830)
                    .portToForwardPipeline(5831)
                    .limeLight4Config(LimelightIOConfig.Limelight4Config.builder().build())
                    .build();

    public static DeviationParamSources asDeviationParams() {
        return new DeviationParamSources() {
            @Override
            public double xStdDev() {
                return LimelightParamsNT.xStdDev.getValue();
            }

            @Override
            public double yStdDev() {
                return LimelightParamsNT.yStdDev.getValue();
            }

            @Override
            public double zStdDev() {
                return LimelightParamsNT.zStdDev.getValue();
            }

            @Override
            public double angleStdDev() {
                return LimelightParamsNT.angleStdDev.getValue();
            }

            @Override
            public double imuCorrectionReliabilityThreshold() {
                return LimelightParamsNT.imuCorrectionReliabilityThreshold.getValue();
            }
        };
    }

    @NTParameter(tableName = "Params/LL")
    public static final class LimelightParams {
        // Vision translation std-devs (meters). Lower = the pose estimator trusts vision MORE and
        // pulls back faster after wheel-slip odometry drift (our failure mode: the robot bumps
        // field elements/robots, wheels slip, dead-reckoning walks the estimate off-field, and
        // vision is what rescues it). The estimator's odometry state std-dev is ~0.1 m, and the
        // effective per-frame vision std-dev is (base * (2 - reliability)), so at 0.3 a typical
        // frame lands ~0.4-0.5 m -- still a few x weaker than odometry, which is intentional so
        // clean driving stays smooth. Lowered 0.7 -> 0.3 to roughly halve post-slip recovery time.
        // Tune live on Params/LL/xStdDev|yStdDev; try 0.2 / 0.15 for more authority, watch for
        // vision jitter during normal driving.
        public static final double xStdDev = 0.3;
        public static final double yStdDev = 0.3;
        public static final double zStdDev = 1.0;
        public static final double angleStdDev = 99999999;
        public static final double imuCorrectionReliabilityThreshold = 0.6;
    }
}
