package lib.ironpulse.swerve;

import lombok.Builder.Default;
import lombok.experimental.SuperBuilder;

@SuperBuilder
public class ImuPigeonConfig {
    /** Sensor acceleration yaw frame; NaN follows the configured mount yaw. */
    @Default public final double accelFrameYaw = Double.NaN;

    public double accelFrameYawDeg() {
        return Double.isFinite(accelFrameYaw) ? accelFrameYaw : mountPoseYaw;
    }

    @Default public final double mountPoseYaw = 0;
    @Default public final double mountPosePitch = 90;
    @Default public final double mountPoseRoll = 0;
    @Default public final double gyroScalarX = 0;
    @Default public final double gyroScalarY = 0;
    @Default public final double gyroScalarZ = 0;
    @Default public final boolean disableNoMotionCalibration = false;
    @Default public final boolean enableCompass = false;
    @Default public final boolean disableTemperatureCompensation = false;
}
