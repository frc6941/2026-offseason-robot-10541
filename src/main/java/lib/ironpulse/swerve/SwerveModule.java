package lib.ironpulse.swerve;

import static edu.wpi.first.units.Units.*;
import static lib.ironpulse.math.MathTools.unwrapAngle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import frc.robot.SwerveModuleParamsNT;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged data;
    private final SwerveConfig swerveConfig;
    private final SwerveConfig.SwerveModuleConfig moduleConfig;
    @Getter private SwerveModulePosition[] odometryPositions;

    /**
     * Steer target latched while the commanded drive speed sits inside {@code
     * swerveConfig.lowSpeedDeadband}, or {@code null} while the module is free to steer.
     */
    private Angle frozenSteerTarget = null;

    public SwerveModule(
            SwerveConfig swerveConfig,
            SwerveConfig.SwerveModuleConfig moduleConfig,
            SwerveModuleIO io) {
        // initialize
        this.io = io;
        this.swerveConfig = swerveConfig;
        this.moduleConfig = moduleConfig;
        this.data = new SwerveModuleIOInputsAutoLogged();
    }

    /**
     * Helper method to update drive controller parameters (PID + FF) by reading current
     * TunableNumber values
     */
    private void updateDriveController() {
        // Through the same bounded readers the composer and the log line use, so the Slot0 config
        // and Compose/params/drive* cannot describe different gains (identity at the defaults).
        double kp = Swerve.driveKP();
        double ki = SwerveModuleParamsNT.Drive.kI.getValue();
        double kd = SwerveModuleParamsNT.Drive.kD.getValue();
        double ks = Swerve.driveKS();
        double kv = Swerve.driveKV();
        double ka = SwerveModuleParamsNT.Drive.kA.getValue();
        io.configDriveController(kp, ki, kd, ks, kv, ka);
    }

    /**
     * Helper method to update steer controller parameters (PID + static friction) by reading
     * current TunableNumber values
     */
    private void updateSteerController() {
        double kp = SwerveModuleParamsNT.Steer.kP.getValue();
        double ki = SwerveModuleParamsNT.Steer.kI.getValue();
        double kd = SwerveModuleParamsNT.Steer.kD.getValue();
        double ks = SwerveModuleParamsNT.Steer.kS.getValue();
        io.configSteerController(kp, ki, kd, ks);
    }

    public void setDriveBrake(boolean isBrake) {
        io.configDriveBrake(isBrake);
    }

    public void updateInputs() {
        io.updateInputs(data);
        Logger.processInputs(swerveConfig.name + "/Module/" + moduleConfig.name, data);
    }

    public void periodic() {
        // compute position for odometry
        int sampleCount =
                Math.min(
                        data.driveMotorPositionRadSamples.length,
                        data.steerMotorPositionRadSamples.length);
        odometryPositions = new SwerveModulePosition[sampleCount];

        for (int i = 0; i < sampleCount; i++)
            odometryPositions[i] =
                    new SwerveModulePosition(
                            data.driveMotorPositionRadSamples[i]
                                    * swerveConfig.wheelDiameter.in(Meter)
                                    * 0.5,
                            new Rotation2d(data.steerMotorPositionRadSamples[i]));

        // run dynamic parameter updates
        if (SwerveModuleParamsNT.Drive.isAnyChanged()) updateDriveController();
        if (SwerveModuleParamsNT.Steer.isAnyChanged()) updateSteerController();

        Logger.recordOutput(
                swerveConfig.name + "/Module/" + moduleConfig.name + "/SteerFrozen",
                frozenSteerTarget != null);
    }

    /** Whether a commanded state's drive speed magnitude sits inside the low speed deadband. */
    private boolean isInsideLowSpeedDeadband(SwerveModuleState state) {
        return Math.abs(state.speedMetersPerSecond)
                < swerveConfig.lowSpeedDeadband.in(MetersPerSecond);
    }

    /**
     * Deadbands a commanded module drive speed: any magnitude inside {@code
     * swerveConfig.lowSpeedDeadband} becomes exactly zero, so the drive motor is not asked to chase
     * near-zero velocity noise. Speeds outside the deadband pass through untouched — this is a hard
     * cut rather than a rescaled deadband, so closed loop velocity tracking is not distorted.
     */
    private LinearVelocity deadbandDriveSpeed(SwerveModuleState state) {
        return isInsideLowSpeedDeadband(state)
                ? MetersPerSecond.zero()
                : MetersPerSecond.of(state.speedMetersPerSecond);
    }

    /**
     * Resolves the steer target for a commanded state, applying the azimuth freeze latch: the first
     * tick whose drive speed falls inside {@code swerveConfig.lowSpeedDeadband} latches the steer
     * target commanded then, and every following sub-threshold tick keeps commanding that same
     * target, so the azimuth holds still instead of tracking the direction of near-zero velocity
     * noise. The latch releases as soon as the speed leaves the deadband.
     */
    private Angle resolveSteerTarget(SwerveModuleState state) {
        Angle steerTarget = state.angle.getMeasure();
        if (isInsideLowSpeedDeadband(state)) {
            if (frozenSteerTarget == null) frozenSteerTarget = steerTarget;
            return frozenSteerTarget;
        }
        frozenSteerTarget = null;
        return steerTarget;
    }

    public void runState(SwerveModuleState state) {
        io.setDriveVelocity(deadbandDriveSpeed(state));
        io.setSteerAngleAbsolute(resolveSteerTarget(state));
    }

    public void runState(SwerveModuleState state, Current ff) {
        io.setDriveVelocity(deadbandDriveSpeed(state), ff);
        io.setSteerAngleAbsolute(resolveSteerTarget(state));
    }

    /**
     * Software-loop drive: the drive motor takes a raw torque current composed on the RIO, while
     * the steer target still follows the commanded state through the same azimuth-freeze latch the
     * velocity path uses. Only the drive command changes topology; steering does not.
     */
    public void runStateCurrent(SwerveModuleState state, Current driveCurrent) {
        runStateCurrent(state, driveCurrent, false);
    }

    /** Steer coupling deliberately turns stopped wheels; it must release the noise freeze latch. */
    public void runStateCurrent(SwerveModuleState state, Current driveCurrent, boolean forceSteer) {
        io.setDriveCurrent(driveCurrent);
        if (forceSteer) frozenSteerTarget = null;
        io.setSteerAngleAbsolute(forceSteer ? state.angle.getMeasure() : resolveSteerTarget(state));
    }

    /**
     * Runs a state with the azimuth freeze latch released. Deliberate zero-speed azimuth commands —
     * the X lock — must still be able to move the steer target even though their drive setpoint
     * sits inside the low speed deadband.
     */
    public void runStateForced(SwerveModuleState state) {
        frozenSteerTarget = null;
        io.setDriveVelocity(deadbandDriveSpeed(state));
        io.setSteerAngleAbsolute(state.angle.getMeasure());
    }

    public void runDriveVoltage(Voltage voltage) {
        frozenSteerTarget = null;
        io.setDriveOpenLoop(voltage);
        io.setSteerAngleAbsolute(Rotation2d.kZero.getMeasure());
    }

    public void runStop() {
        frozenSteerTarget = null;
        io.setDriveVelocity(MetersPerSecond.zero());
        io.setSteerOpenLoop(Volts.zero());
    }

    public Distance getDriveDistance() {
        return swerveConfig.wheelDiameter.times(data.driveMotorPositionRad * 0.5);
    }

    public LinearVelocity getDriveVelocity() {
        return MetersPerSecond.of(
                data.driveMotorVelocityRadPerSec * 0.5 * swerveConfig.wheelDiameter.in(Meter));
    }

    /**
     * Measured drive ROTOR speed, rad/s. The inputs carry mechanism (wheel) speed, so the gear
     * ratio goes back on: back EMF is a property of the rotor, not the wheel.
     */
    public double getDriveRotorVelocityRadPerSec() {
        return data.driveMotorVelocityRadPerSec * swerveConfig.driveGearRatio;
    }

    /** Measured drive applied voltage — the numerator of this module's duty. */
    public double getDriveAppliedVolts() {
        return data.driveMotorVoltageVolt;
    }

    /** Measured drive supply current — what the module actually draws from the pack. */
    public double getDriveSupplyCurrentAmpere() {
        return data.driveMotorSupplyCurrentAmpere;
    }

    public double getDriveTorqueCurrentAmpere() {
        return data.driveMotorTorqueCurrentAmpere;
    }

    public Angle getSteerAngle() {
        return Radian.of(data.steerMotorPositionRad);
    }

    public AngularVelocity getSteerAngularVelocity() {
        return RadiansPerSecond.of(data.steerMotorVelocityRadPerSec);
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(
                getDriveVelocity(), new Rotation2d(unwrapAngle(0.0, getSteerAngle().in(Radian))));
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getDriveDistance(), new Rotation2d(getSteerAngle()));
    }

    public SwerveModulePosition[] getSampledSwerveModulePositions() {
        int sampleCount =
                Math.min(
                        data.driveMotorPositionRadSamples.length,
                        data.steerMotorPositionRadSamples.length);
        SwerveModulePosition[] positions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++)
            positions[i] =
                    new SwerveModulePosition(
                            swerveConfig.wheelDiameter.times(
                                    data.driveMotorPositionRadSamples[i] * 0.5),
                            new Rotation2d(data.steerMotorPositionRadSamples[i]));
        return positions;
    }
}
