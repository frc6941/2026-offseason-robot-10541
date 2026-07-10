package frc.robot.subsystems.Limelight;

import lib.ironpulse.limelight.DeviationParamSources;
import lib.ntext.NTParameter;

public class LimelightParams {

    // Vision covariance (standard deviations) and IMU-correction threshold consumed by
    // LimelightIOReal via DeviationParamSources. Exposed over NetworkTables so they can be
    // tuned live on the real robot.
    // TODO: tune vision std-devs on the real robot.
    @NTParameter(tableName = "Params/Limelight")
    public static final class LimelightDeviationParams {
        public static final double xStdDev = 0.7;
        public static final double yStdDev = 0.7;
        public static final double zStdDev = 9999.0;
        public static final double angleStdDev = 999999999.0;
        public static final double imuCorrectionReliabilityThreshold = 0.9;
    }

    /**
     * Build a {@link DeviationParamSources} backed by the live NetworkTables values above. Each
     * accessor reads {@code getValue()} on every call, so edits from the dashboard take effect
     * without a redeploy.
     */
    public static DeviationParamSources deviationParamSources() {
        return new DeviationParamSources() {
            public double xStdDev() {
                return LimelightDeviationParamsNT.xStdDev.getValue();
            }

            public double yStdDev() {
                return LimelightDeviationParamsNT.yStdDev.getValue();
            }

            public double zStdDev() {
                return LimelightDeviationParamsNT.zStdDev.getValue();
            }

            public double angleStdDev() {
                return LimelightDeviationParamsNT.angleStdDev.getValue();
            }

            public double imuCorrectionReliabilityThreshold() {
                return LimelightDeviationParamsNT.imuCorrectionReliabilityThreshold.getValue();
            }
        };
    }
}
