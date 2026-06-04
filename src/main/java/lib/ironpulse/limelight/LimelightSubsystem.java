package lib.ironpulse.limelight;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import lib.ironpulse.math.MathTools;
import lib.ironpulse.utils.LoggedTracer;
import org.littletonrobotics.junction.Logger;

public class LimelightSubsystem extends SubsystemBase {
    private final HashMap<String, LimelightIO> ioNames = new HashMap<>();
    private final HashMap<LimelightIO, LimelightIOInputsAutoLogged> ios = new HashMap<>();
    private final Localizable localizationProvider;
    private int suppressVisionFrames = 0;

    public LimelightSubsystem(Localizable localizationProvider, LimelightIO... ios) {
        super("Limelight");
        this.localizationProvider = localizationProvider;
        for (LimelightIO io : ios) {
            this.ios.put(io, new LimelightIOInputsAutoLogged());
            this.ioNames.put(io.getName(), io);
        }
    }

    private Matrix<N4, N1> getVisionStdDev(LimelightIO io, double reliability) {
        double[] stdDev = io.getVisionStdDevComponents(reliability);
        return VecBuilder.fill(stdDev[0], stdDev[1], stdDev[2], stdDev[3]);
    }

    private void addVisionMeasurement() {
        if (suppressVisionFrames > 0) {
            suppressVisionFrames--;
            return;
        }

        for (Map.Entry<LimelightIO, LimelightIOInputsAutoLogged> entry : ios.entrySet()) {
            LimelightIO io = entry.getKey();
            LimelightIOInputsAutoLogged input = entry.getValue();
            if (MathTools.epsilonEquals(input.reliability, 0)) {
                continue;
            }
            localizationProvider.addVisionMeasurement(
                    input.pose, input.timestampSeconds, getVisionStdDev(io, input.reliability));
        }
    }

    public void requestInternalIMUReseedAll() {
        suppressVisionFrames = Math.max(suppressVisionFrames, 5);
        for (LimelightIO io : ios.keySet()) {
            io.requestInternalIMUReseed();
        }
    }

    @Override
    public void periodic() {
        for (Map.Entry<LimelightIO, LimelightIOInputsAutoLogged> entry : ios.entrySet()) {
            LimelightIO io = entry.getKey();
            io.setRobotOrientation();
            LimelightIOInputsAutoLogged inputs = entry.getValue();

            io.updateInputs(inputs);
            Logger.processInputs("Limelight/" + io.getName(), inputs);

            if (!io.canUseInternalIMU()) {
                continue;
            }

            Logger.recordOutput("Limelight/IMU/" + io.getName() + "_INT", io.getIMUYawInternal());
            Logger.recordOutput("Limelight/IMU/" + io.getName() + "_ROBOT", io.getIMUYawRobot());

            SmartDashboard.putBoolean(
                    "Limelight/" + io.getName() + "alive",
                    Objects.equals(inputs.status, "Connected"));
        }

        addVisionMeasurement();
        LoggedTracer.record("Limelight");
    }

    private LimelightIO getIoById(String id) {
        LimelightIO io = ioNames.getOrDefault(id, null);
        if (io == null) {
            throw new IllegalArgumentException(id + " is not a valid limelight");
        }
        return io;
    }

    public void setPipeline(String id, int pipeline) {
        getIoById(id).setPipeline(pipeline);
    }

    public int getPipeline(String id) {
        return getIoById(id).getPipeline();
    }

    public void setLEDMode(String id, LimelightIO.LEDMode mode) {
        getIoById(id).setLEDMode(mode);
    }

    public void setAprilTagIdFilter(String id, int[] tagIds) {
        getIoById(id).setAprilTagIdFilter(tagIds);
    }

    public void clearAprilTagIdFilter(String id) {
        getIoById(id).clearAprilTagIdFilter();
    }

    public void setThrottleAll(boolean enabled) {
        for (LimelightIO io : ios.keySet()) {
            io.setThrottle(enabled);
        }
    }

    public Pose2d getPose(String id) {
        LimelightIOInputsAutoLogged inputs = ios.get(getIoById(id));
        return inputs.pose == null ? new Pose2d() : inputs.pose.toPose2d();
    }
}
