package lib.ironpulse.limelight;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lib.ironpulse.utils.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

/**
 * Limelight IO. Takes in a LimelightConfig and odometry from swerve for enhanced detection. Should
 * be instantiated for every limelight.
 */
public class LimelightIOReal implements LimelightIO {
    public final LimelightIOConfig config;
    private final DoubleSupplier yawSupplier;
    private final DoubleSupplier yawVelocitySupplier;
    private final BooleanSupplier rejectionSupplier;
    private final DeviationParamSources deviationParams;
    private boolean isPrevDisabled = true;
    private double lastHeartbeat = -1;
    private double lastTsBootMs = -1;
    private double lastSeenTime = 0.0;
    private double lastPoseFrameSignature = Double.NaN;
    private double latestPoseFrameSignature = Double.NaN;
    private int reseedFramesRemaining = 0;

    public LimelightIOReal(
            LimelightIOConfig config,
            DoubleSupplier yawSupplier,
            DoubleSupplier yawVelocitySupplier,
            BooleanSupplier rejectionSupplier,
            DeviationParamSources deviationParams) {
        this.config = config;
        this.yawSupplier = yawSupplier;
        this.yawVelocitySupplier = yawVelocitySupplier;
        this.rejectionSupplier = rejectionSupplier;
        this.deviationParams = deviationParams;
        if (useInternalIMU()) {
            LimelightHelpers.SetIMUAssistAlpha(config.getName(), config.getFilterAlpha());
            setIMUMode();
        }
        if (config.getPortToForwardStream() != 0)
            PortForwarder.add(config.getPortToForwardStream(), config.getName() + ".local", 5800);
        if (config.getPortToForwardPipeline() != 0)
            PortForwarder.add(config.getPortToForwardPipeline(), config.getName() + ".local", 5801);
    }

    private static double getSpanReliability(double span, double distance, double area) {
        // --- span reliability ---
        double rSpan;
        if (span <= 0.0) {
            rSpan = 0.0;
        } else if (span <= 0.3) {
            // ramp from 0.6 at span=0 to 1.0 at span=0.3
            rSpan = 0.6 + 0.4 * (span / 0.3);
        } else if (span <= 0.5) {
            // ideal plateau
            rSpan = 1.0;
        } else if (span <= 1.0) {
            // gentle decay from 1.0 at 0.5 to 0.3 at 1.0
            rSpan = 1.0 - 0.7 * ((span - 0.5) / 0.5);
        } else if (span <= 2.0) {
            // further decay from 0.3 at 1.0 down towards 0.05 at 2.0 (capped)
            rSpan = 0.3 - 0.25 * (span - 1.0);
            if (rSpan < 0.05) rSpan = 0.05;
        } else {
            rSpan = 0.05;
        }

        // --- distance reliability ---
        double rDist = 1.0 / (1.0 + (distance / 3.0));

        // --- area reliability ---
        double normArea = (area - 0.1) / 0.9; // map [0.1, 1.0] -> [0, 1]
        if (normArea < 0.0) normArea = 0.0;
        if (normArea > 1.0) normArea = 1.0;

        // sqrt: boosts medium values a bit, softer penalty for not-huge areas
        double rArea = Math.sqrt(normArea);

        double raw = (rSpan + rDist + rArea) / 3.0;
        // --- combine using geometric mean ---
        double reliability = Math.pow(raw, 0.8);
        return Math.max(0.0, Math.min(1.0, reliability));
    }

    private String updateStatus(LimelightHelpers.PoseEstimate estimate) {
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        double hb = LimelightHelpers.getHeartbeat(config.getName());

        boolean seenHeartbeat = hb != lastHeartbeat;
        boolean poseFrameUpdated =
                estimate.tagCount > 0
                        && Double.isFinite(latestPoseFrameSignature)
                        && (!Double.isFinite(lastPoseFrameSignature)
                                || Double.compare(latestPoseFrameSignature, lastPoseFrameSignature)
                                        != 0);

        if (seenHeartbeat || poseFrameUpdated) {
            double tsBootMs =
                    LimelightHelpers.getLatestResults(config.getName()).timestamp_LIMELIGHT_publish;
            if (tsBootMs <= 0.0) {
                tsBootMs = estimate.timestampSeconds * 1000.0;
            }

            lastHeartbeat = hb;
            lastTsBootMs = tsBootMs;
            lastPoseFrameSignature = latestPoseFrameSignature;
            lastSeenTime = now;
        }

        if (now - lastSeenTime > 0.5) {
            return "Disconnected";
        }
        return "Connected";
    }

    private boolean isLimelight4() {
        return config.getLimeLight4Config() != null;
    }

    private boolean useInternalIMU() {
        return isLimelight4() && config.getLimeLight4Config().isUseInternalIMU();
    }

    @Override
    public boolean canUseInternalIMU() {
        return config.getMountPosition() == LimelightIOConfig.MountPosition.ON_ROBOT
                && useInternalIMU();
    }

    /**
     * The reliability score ranges from 0 to 1 (inclusive) and indicates the trustworthiness of
     * this limelight's prediction accounting its prediction weight.
     *
     * @return 0-1 (inclusive). 1 is the most reliable and 0 the least.
     */
    @Override
    public double getReliabilityScore(LimelightHelpers.PoseEstimate poseEstimate) {
        double distance = poseEstimate.avgTagDist;
        double area = poseEstimate.avgTagArea;
        double span = poseEstimate.tagSpan;

        // hard reject
        if (distance > 5.5 || area < 0.05 || rejectionSupplier.getAsBoolean()) return 0.0;

        // --- span reliability ---
        // ideal span: [0.3, 0.5] = 1.0
        double reliability = getSpanReliability(span, distance, area);

        if (poseEstimate.tagCount == 0) {
            // no tags
            return 0;
        } else if (poseEstimate.tagCount == 1) {
            return 0.75 * config.getWeight() * reliability;
        } else {
            // tags >= 2
            return 1 * config.getWeight() * reliability;
        }
    }

    @Override
    public String getName() {
        return config.getName();
    }

    /**
     * Switches the internal IMU between states.
     *
     * <p>Context: called upon initialization and change of robot states (Disabled <-> Enabled)
     */
    private void setIMUMode() {
        if (!canUseInternalIMU()) {
            // I don't really think internal IMU would be stable on limelights
            // mounted on mechanisms (e.g. turrets, shooters) as the yaw reading
            // would be different from that of the robot.
            // So we do not trust the internal IMU completely...
            LimelightHelpers.SetIMUMode(config.getName(), InternalIMUMode.EXTERNAL_ONLY.getValue());
            Logger.recordOutput("Limelight/IMU/Mode", "external");
            return;
        }
        if (RobotState.isDisabled() || reseedFramesRemaining > 0) {
            // disabled; use IMU mode 1 - seed internal IMU with data
            LimelightHelpers.SetIMUMode(config.getName(), InternalIMUMode.EXTERNAL_SEED.getValue());
            Logger.recordOutput("Limelight/IMU/Mode", "seed");
        } else {
            // enabled - keep using the robot's external IMU for this project
            LimelightHelpers.SetIMUMode(config.getName(), InternalIMUMode.EXTERNAL_ONLY.getValue());
            Logger.recordOutput("Limelight/IMU/Mode", "external_only");
        }
    }

    @Override
    public void requestInternalIMUReseed() {
        if (!canUseInternalIMU() || RobotState.isDisabled()) {
            return;
        }
        reseedFramesRemaining = 3;
        setIMUMode();
    }

    @Override
    public void setLEDMode(LEDMode mode) {
        switch (mode) {
            case ON -> LimelightHelpers.setLEDMode_ForceOn(config.getName());
            case OFF -> LimelightHelpers.setLEDMode_ForceOff(config.getName());
            case BLINK -> LimelightHelpers.setLEDMode_ForceBlink(config.getName());
            case PIPELINE_CONTROL -> LimelightHelpers.setLEDMode_PipelineControl(config.getName());
        }
    }

    @Override
    public int getPipeline() {
        return (int) LimelightHelpers.getCurrentPipelineIndex(config.getName());
    }

    @Override
    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(config.getName(), pipeline);
    }

    @Override
    public void setThrottle(boolean robotEnabled) {
        if (!isLimelight4()) {
            return;
        }
        LimelightHelpers.SetThrottle(
                config.getName(),
                robotEnabled
                        ? config.getLimeLight4Config().getThrottleWhenEnabled()
                        : config.getLimeLight4Config().getThrottleWhenDisabled());
    }

    @Override
    public void setAprilTagIdFilter(int[] ids) {
        LimelightHelpers.SetFiducialIDFiltersOverride(config.getName(), ids);
    }

    public void setRobotOrientation() {
        LimelightHelpers.SetRobotOrientation(
                config.getName(),
                yawSupplier.getAsDouble(),
                yawVelocitySupplier.getAsDouble(),
                0,
                0,
                0,
                0); // the last 5 parameters are not necessary
        Logger.recordOutput("Limelight/IMU/Swerve", yawSupplier.getAsDouble());
    }

    private LimelightHelpers.PoseEstimate getPoseEstimate() {
        String entryName = config.isUseMegaTag2() ? "botpose_orb_wpiblue" : "botpose_wpiblue";
        double[] poseArray =
                LimelightHelpers.getLimelightNTDoubleArray(config.getName(), entryName);
        latestPoseFrameSignature = poseFrameSignature(poseArray);
        Logger.recordOutput("Limelight/" + config.getName() + "/PoseArrayLength", poseArray.length);

        if (poseArray.length < 11) {
            return config.isUseMegaTag2()
                    ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.getName())
                    : LimelightHelpers.getBotPoseEstimate_wpiBlue(config.getName());
        }

        double latencyMs = extractArrayEntry(poseArray, 6);
        int tagCount = (int) extractArrayEntry(poseArray, 7);
        double tagSpan = extractArrayEntry(poseArray, 8);
        double tagDist = extractArrayEntry(poseArray, 9);
        double tagArea = extractArrayEntry(poseArray, 10);
        int valsPerFiducial = 7;
        int expectedTotalVals = 11 + valsPerFiducial * tagCount;
        LimelightHelpers.RawFiducial[] rawFiducials;

        if (tagCount <= 0 || poseArray.length != expectedTotalVals) {
            rawFiducials = new LimelightHelpers.RawFiducial[0];
        } else {
            rawFiducials = new LimelightHelpers.RawFiducial[tagCount];
            for (int i = 0; i < tagCount; i++) {
                int baseIndex = 11 + i * valsPerFiducial;
                rawFiducials[i] =
                        new LimelightHelpers.RawFiducial(
                                (int) poseArray[baseIndex],
                                poseArray[baseIndex + 1],
                                poseArray[baseIndex + 2],
                                poseArray[baseIndex + 3],
                                poseArray[baseIndex + 4],
                                poseArray[baseIndex + 5],
                                poseArray[baseIndex + 6]);
            }
        }

        return new LimelightHelpers.PoseEstimate(
                LimelightHelpers.toPose2D(poseArray),
                Timer.getFPGATimestamp() - latencyMs / 1000.0,
                latencyMs,
                tagCount,
                tagSpan,
                tagDist,
                tagArea,
                rawFiducials,
                config.isUseMegaTag2());
    }

    private static double extractArrayEntry(double[] values, int index) {
        return values.length > index ? values[index] : 0.0;
    }

    private static double poseFrameSignature(double[] values) {
        if (values.length < 11) {
            return Double.NaN;
        }
        double signature = values.length;
        for (int i = 0; i < values.length; i++) {
            signature += values[i] * (i + 1);
        }
        return signature;
    }

    @Override
    public void updateInputs(LimelightIOInputs inputs) {
        if (canUseInternalIMU()) {
            if (isPrevDisabled != RobotState.isDisabled()) {
                setIMUMode();
            } else if (reseedFramesRemaining > 0) {
                reseedFramesRemaining--;
                if (reseedFramesRemaining == 0 && !RobotState.isDisabled()) {
                    setIMUMode();
                }
            }
        }
        isPrevDisabled = RobotState.isDisabled();

        LimelightHelpers.PoseEstimate estimate = getPoseEstimate();
        inputs.pose = new Pose3d(estimate.pose);
        inputs.timestampSeconds = estimate.timestampSeconds;
        inputs.latency = estimate.latency;
        inputs.detectedTagIds = Arrays.stream(estimate.rawFiducials).mapToInt(r -> r.id).toArray();
        inputs.reliability = getReliabilityScore(estimate);
        inputs.tagSpan = estimate.tagSpan;
        inputs.avgTagArea = estimate.avgTagArea;
        inputs.avgTagDist = estimate.avgTagDist;
        inputs.status = updateStatus(estimate);
        inputs.lastHeartbeat = lastHeartbeat;
        inputs.lastTsBootMs = lastTsBootMs;
        inputs.lastSeenTime = lastSeenTime;
        inputs.temperature = LimelightHelpers.getTemperature(config.getName());
    }

    @Override
    public double[] getVisionStdDevComponents(double reliability) {
        return new double[] {
            deviationParams.xStdDev() * (2 - reliability),
            deviationParams.yStdDev() * (2 - reliability),
            deviationParams.zStdDev() * (2 - reliability),
            deviationParams.angleStdDev() * (2 - reliability)
        };
    }

    @Override
    public double getImuCorrectionReliabilityThreshold() {
        return deviationParams.imuCorrectionReliabilityThreshold();
    }

    @Override
    public double getIMUYawInternal() {
        return LimelightHelpers.getIMUData(config.getName()).Yaw;
    }

    @Override
    public double getIMUYawRobot() {
        return LimelightHelpers.getIMUData(config.getName()).robotYaw;
    }
}
