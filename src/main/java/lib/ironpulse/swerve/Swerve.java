package lib.ironpulse.swerve;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import lib.ironpulse.limelight.Localizable;
import lib.ironpulse.utils.LoggedTracer;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase implements Localizable {
    // locks
    static final Lock odometryLock = new ReentrantLock();
    // config and io
    private final SwerveConfig config;
    private final List<SwerveModule> modules;
    private final ImuIO imuIO;
    // controller
    private final SwerveDriveKinematics kinematics;
    private final SwerveSetpointGenerator setpointGenerator;
    // estimator
    private final SwerveDrivePoseEstimator3d poseEstimator;
    // precomputed
    private final List<Rotation2d> xLockAngles;
    public final ImuIOInputsAutoLogged imuIOInputs;
    private SwerveSetpoint setpointCurr;
    @Getter private Voltage previouslyAppliedVoltage;
    private MODE mode = MODE.VELOCITY;

    public Swerve(SwerveConfig swerveConfig, ImuIO imuIO, SwerveModuleIO... moduleIOs) {
        this.config = swerveConfig;
        this.modules = new ArrayList<>(moduleIOs.length);
        if (config.moduleCount() != moduleIOs.length)
            throw new Error(
                    "Module count mismatch: " + config.moduleCount() + " vs " + moduleIOs.length);

        // ios
        this.imuIO = imuIO;
        this.imuIOInputs = new ImuIOInputsAutoLogged();
        for (int i = 0; i < config.moduleConfigs.length; i++)
            this.modules.add(i, new SwerveModule(config, config.moduleConfigs[i], moduleIOs[i]));
        SwerveModuleState[] states = new SwerveModuleState[config.moduleCount()];
        for (int i = 0; i < config.moduleCount(); i++)
            states[i] = modules.get(i).getSwerveModuleState();

        // kinematics, limits, and setpoint generator
        kinematics = new SwerveDriveKinematics(config.moduleLocations());
        setpointGenerator =
                SwerveSetpointGenerator.builder()
                        .kinematics(kinematics)
                        .chassisLimit(config.defaultSwerveLimit)
                        .moduleLimit(config.defaultSwerveModuleLimit)
                        .build();
        setpointCurr = new SwerveSetpoint(new ChassisSpeeds(), states);

        // estimator
        poseEstimator =
                new SwerveDrivePoseEstimator3d(
                        kinematics, new Rotation3d(), getModulePositions(), new Pose3d());

        // precompute
        var moduleLocations = config.moduleLocations();
        xLockAngles = new ArrayList<>(config.moduleCount());
        for (int i = 0; i < config.moduleCount(); i++)
            xLockAngles.add(i, moduleLocations[i].getAngle());
    }

    // ------- Core Methods -------
    @Override
    public void periodic() {
        // io updates
        odometryLock.lock();
        imuIOInputs.yawVelocityRadPerSecCmd = getChassisSpeeds().omegaRadiansPerSecond;
        imuIO.updateInputs(imuIOInputs);
        Logger.processInputs(config.name + "/IMU", imuIOInputs);
        modules.forEach(
                module -> {
                    module.updateInputs();
                    module.periodic();
                });

        // odom
        var swerveModulePositionsWithTime = getSampledModulePositions();
        var rotations = imuIOInputs.odometryRotations;
        var now = Timer.getTimestamp();
        for (int i = 0; i < swerveModulePositionsWithTime.size(); i++) {
            var positionWithTime = swerveModulePositionsWithTime.get(i);
            poseEstimator.updateWithTime(
                    now,
                    rotations[i], // FIXME: there's a discrepancy between Phoenix time and rio time.
                    // need to find
                    // the offset. this fix is temporary
                    positionWithTime.getSecond());
        }
        odometryLock.unlock();
        LoggedTracer.record(config.name + "/Inputs");

        // telemetry
        Logger.recordOutput(config.name + "/Mode", mode);
        Logger.recordOutput(config.name + "/ChassisSpeedCurr", getChassisSpeeds());
        Logger.recordOutput(config.name + "/SwerveModuleStateCurr", getModuleStates());
        Logger.recordOutput(config.name + "/SwerveModuleStateCmd", setpointCurr.moduleStates());
        Logger.recordOutput(config.name + "/ChassisSpeedCmd", setpointCurr.chassisSpeeds());
        Logger.recordOutput(
                config.name + "/SwerveEstimatorPose", poseEstimator.getEstimatedPosition());

        var limit = getSwerveLimit();
        Logger.recordOutput(
                config.name + "/Limit/MaxLinVelMps", limit.maxLinearVelocity().in(MetersPerSecond));
        Logger.recordOutput(
                config.name + "/Limit/MaxSkidAccMps2",
                limit.maxSkidAcceleration().in(MetersPerSecondPerSecond));
        Logger.recordOutput(
                config.name + "/Limit/MaxAngvelDegps",
                limit.maxAngularVelocity().in(DegreesPerSecond));
        Logger.recordOutput(
                config.name + "/Limit/MaxAngAccDegps2",
                limit.maxAngularAcceleration().in(DegreesPerSecondPerSecond));

        var wrench = getWrench();
        Logger.recordOutput(config.name + "/Wrench/FxN", wrench[0]);
        Logger.recordOutput(config.name + "/Wrench/FyN", wrench[1]);
        Logger.recordOutput(config.name + "/Wrench/TauNm", wrench[2]);
    }

    // -------- Run -------

    /**
     * Run a twist for the swerve drive.
     *
     * @param VRT the desired twist. note twist is expressed under the robot frame. if want to use
     *     field oriented drive,need to do frame transform elsewhere before pass in the command.
     */
    public void runTwist(ChassisSpeeds VRT) {
        mode = MODE.VELOCITY;
        setpointCurr = setpointGenerator.generate(VRT, setpointCurr, config.dtS);

        for (int i = 0; i < config.moduleCount(); i++)
            modules.get(i).runState(setpointCurr.moduleStates()[i]);
    }

    public void runTwistWithTorque(ChassisSpeeds VRT, Current[] tau) {
        assert (tau.length == config.moduleCount());
        mode = MODE.VELOCITY;
        setpointCurr = setpointGenerator.generate(VRT, setpointCurr, config.dtS);

        for (int i = 0; i < config.moduleCount(); i++)
            modules.get(i).runState(setpointCurr.moduleStates()[i], tau[i]);
    }

    public void runVoltage(Voltage voltage) {
        mode = MODE.VOLTAGE;
        previouslyAppliedVoltage = voltage;
        for (int i = 0; i < config.moduleCount(); i++) modules.get(i).runDriveVoltage(voltage);
    }

    public void runStop() {
        runVoltage(Volt.of(0.0));
    }

    public void runStopAndLock() {
        mode = MODE.VELOCITY;
        SwerveModuleState[] lockStates = new SwerveModuleState[config.moduleCount()];
        for (int i = 0; i < config.moduleCount(); i++) {
            lockStates[i] = new SwerveModuleState(0.0, xLockAngles.get(i));
            modules.get(i).runState(lockStates[i]);
        }
        kinematics.resetHeadings(xLockAngles.toArray(new Rotation2d[0]));
        setpointCurr = new SwerveSetpoint(new ChassisSpeeds(), lockStates);
    }

    // ------- Getters -------
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.size()];
        for (int i = 0; i < modules.size(); i++) states[i] = modules.get(i).getSwerveModuleState();
        return states;
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[modules.size()];
        for (int i = 0; i < modules.size(); i++)
            states[i] = modules.get(i).getSwerveModulePosition();
        return states;
    }

    public List<Pair<Double, SwerveModulePosition[]>> getSampledModulePositions() {
        double[] timestamps = imuIOInputs.odometryYawTimestamps;
        int moduleCount = modules.size();

        // cache each module’s sampled positions array
        List<SwerveModulePosition[]> samplesByModule =
                modules.stream().map(SwerveModule::getSampledSwerveModulePositions).toList();

        // The IMU and per-module sample queues are filled/drained independently by the odometry
        // thread and can momentarily differ in length. Clamp to the smallest common count so we
        // never index past a shorter array (was an AIOOBE crash once the sampler runs).
        // TODO(lib-IP-2026): upstream this guard (or read all queues under one lock).
        int sampleCount = timestamps.length;
        for (SwerveModulePosition[] moduleSamples : samplesByModule)
            sampleCount = Math.min(sampleCount, moduleSamples.length);

        List<Pair<Double, SwerveModulePosition[]>> result = new ArrayList<>(sampleCount);
        for (int sampleIdx = 0; sampleIdx < sampleCount; sampleIdx++) {
            // build the array of positions at this timestamp
            SwerveModulePosition[] positionsAtTime = new SwerveModulePosition[moduleCount];
            for (int moduleIdx = 0; moduleIdx < moduleCount; moduleIdx++)
                positionsAtTime[moduleIdx] = samplesByModule.get(moduleIdx)[sampleIdx];
            result.add(new Pair<>(timestamps[sampleIdx], positionsAtTime));
        }

        return result;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getChassisSpeedsCmd() {
        if (mode != MODE.VELOCITY) return new ChassisSpeeds();
        return setpointCurr.chassisSpeeds();
    }

    /**
     * Returns the chassis wrench as {@code [Fx (N), Fy (N), τ (N·m)]} in robot frame.
     *
     * <p>Computed by abusing {@link SwerveDriveKinematics#toChassisSpeeds}: each module's wheel
     * force (torque-current × Kt × gear-ratio / wheel-radius) is fed in as a speed, so the
     * kinematics least-squares projection maps wheel forces onto the chassis wrench space.
     */
    public double[] getWrench() {
        double wheelRadius = config.wheelDiameter.in(Meter) * 0.5;
        SwerveModuleState[] forceStates = new SwerveModuleState[modules.size()];
        for (int i = 0; i < modules.size(); i++) {
            double force =
                    modules.get(i).getDriveTorqueCurrentAmpere()
                            * config.driveMotorKt
                            * config.driveGearRatio
                            / wheelRadius;
            forceStates[i] =
                    new SwerveModuleState(
                            force, new Rotation2d(modules.get(i).getSteerAngle().in(Radian)));
        }
        ChassisSpeeds w = kinematics.toChassisSpeeds(forceStates);
        return new double[] {w.vxMetersPerSecond, w.vyMetersPerSecond, w.omegaRadiansPerSecond};
    }

    public Pose3d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetEstimatedPose(Pose3d pose) {
        odometryLock.lock();
        // Reset IMU hardware to match the new pose rotation
        double newYawDegrees = pose.getRotation().toRotation2d().getDegrees();
        imuIO.setYawDeg(newYawDegrees);
        // Reset pose estimator with the new pose
        // Use the new rotation from pose parameter, not stale imuIOInputs.odometryRotations[0]
        Rotation3d newRotation = pose.getRotation();
        poseEstimator.resetPosition(newRotation, getModulePositions(), pose);
        odometryLock.unlock();
    }

    public Optional<Pose3d> getEstimatedPoseAt(Time time) {
        return poseEstimator.sampleAt(time.in(Seconds));
    }

    @Override
    public void addVisionMeasurement(
            Pose3d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N4, N1> visionMeasurementStdDevs) {
        Logger.recordOutput(config.name + "/VisionCorrectionPose", visionRobotPoseMeters);
        Logger.recordOutput(config.name + "/VisionCorrectionTimestampSeconds", timestampSeconds);
        poseEstimator.addVisionMeasurement(
                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    @Override
    public double getIMUYaw() {
        return poseEstimator.getEstimatedPosition().getRotation().toRotation2d().getDegrees();
    }

    public double getYawVelocityRadPerSec() {
        return imuIOInputs.yawVelocityRadPerSec;
    }

    public double getPitchVelocityRadPerSec() {
        return imuIOInputs.pitchVelocityRadPerSec;
    }

    public double getPitchPosRad() {
        return imuIOInputs.pitchPosition.getRadians();
    }

    // ------- Configurations -------
    public SwerveLimit getSwerveLimit() {
        return setpointGenerator.getChassisLimit();
    }

    public void setSwerveLimit(SwerveLimit limit) {
        setpointGenerator.setChassisLimit(limit);
    }

    public void setSwerveLimitDefault() {
        setpointGenerator.setChassisLimit(config.defaultSwerveLimit);
    }

    public SwerveModuleLimit getSwerveModuleLimit() {
        return setpointGenerator.getModuleLimit();
    }

    public void setSwerveModuleLimit(SwerveModuleLimit limit) {
        setpointGenerator.setModuleLimit(limit);
    }

    public void setSwerveModuleLimitDefault() {
        setpointGenerator.setModuleLimit(config.defaultSwerveModuleLimit);
    }

    @Override
    public void setIMUYaw(double yaw) {
        imuIO.setYawDeg(yaw);
    }

    public enum MODE {
        VELOCITY,
        VOLTAGE
    }
}
