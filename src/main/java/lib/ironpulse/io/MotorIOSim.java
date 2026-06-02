package lib.ironpulse.io;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.subsystem.SubsystemConfig.SimConfig;

/**
 * A simple kinematic motor simulation that moves directly according to its limits. This avoids the
 * complexity and potential bugs of a full physics simulation when realism is not required.
 */
public class MotorIOSim implements MotorIO {
    private final SimConfig cfg;
    private final SubsystemConfig subsystemConfig;
    private final DCMotor dcMotor;
    private final double maxVelocityRPS;

    private double lastTimestamp = Timer.getTimestamp();

    private double currentPositionRot = 0.0;
    private double currentVelocityRPS = 0.0;
    private double appliedVolts = 0.0;

    private boolean forwardSoftLimitEnabled;
    private boolean reverseSoftLimitEnabled;

    private TrapezoidProfile profile;
    private TrapezoidProfile.Constraints currentConstraints;
    private TrapezoidProfile.State currentProfileState = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State goalProfileState = new TrapezoidProfile.State(0, 0);

    private enum InternalControlMode {
        VOLTAGE,
        VELOCITY,
        POSITION_CONTROL
    }

    private InternalControlMode controlMode = InternalControlMode.VOLTAGE;
    private double velocitySetpointRPS = 0.0;

    public MotorIOSim(SubsystemConfig cfg) {
        this.cfg = cfg.simConfig;
        this.subsystemConfig = cfg;
        // Assume 1 Kraken X60 by default for free speed constants.
        // In a kinematic simulation without load, the number of motors doesn't affect the max
        // speed.
        this.dcMotor = DCMotor.getKrakenX60Foc(1);
        this.maxVelocityRPS = (dcMotor.freeSpeedRadPerSec / (2 * Math.PI)) / this.cfg.gearRatio;

        this.forwardSoftLimitEnabled = !Double.isNaN(cfg.forwardSoftLimitDegrees.magnitude());
        this.reverseSoftLimitEnabled = !Double.isNaN(cfg.reverseSoftLimitDegrees.magnitude());

        currentConstraints =
                (this.cfg.profile != null)
                        ? this.cfg.profile
                        : new TrapezoidProfile.Constraints(50.0, 100.0);
        this.profile = new TrapezoidProfile(currentConstraints);
    }

    @Override
    public void readInputs(MotorInputs inputs) {
        double now = Timer.getTimestamp();
        double dt = now - lastTimestamp;
        lastTimestamp = now;

        switch (controlMode) {
            case VOLTAGE:
                currentVelocityRPS = (appliedVolts / 12.0) * maxVelocityRPS;
                currentPositionRot += currentVelocityRPS * dt;
                currentProfileState =
                        new TrapezoidProfile.State(currentPositionRot, currentVelocityRPS);
                break;

            case VELOCITY:
                currentVelocityRPS =
                        MathUtil.clamp(velocitySetpointRPS, -maxVelocityRPS, maxVelocityRPS);
                currentPositionRot += currentVelocityRPS * dt;
                currentProfileState =
                        new TrapezoidProfile.State(currentPositionRot, currentVelocityRPS);
                break;

            case POSITION_CONTROL:
                currentProfileState = profile.calculate(dt, currentProfileState, goalProfileState);
                currentPositionRot = currentProfileState.position;
                currentVelocityRPS = currentProfileState.velocity;
                break;
        }

        // // Apply soft limits
        // if (forwardSoftLimitEnabled) {
        //     double threshold = subsystemConfig.forwardSoftLimitDegrees.in(Rotations);
        //     if (currentPositionRot >= threshold) {
        //         currentPositionRot = threshold;
        //         currentVelocityRPS = Math.min(0, currentVelocityRPS);
        //         currentProfileState =
        //                 new TrapezoidProfile.State(currentPositionRot, currentVelocityRPS);
        //     }
        // }
        // if (reverseSoftLimitEnabled) {
        //     double threshold = subsystemConfig.reverseSoftLimitDegrees.in(Rotations);
        //     if (currentPositionRot <= threshold) {
        //         currentPositionRot = threshold;
        //         currentVelocityRPS = Math.max(0, currentVelocityRPS);
        //         currentProfileState =
        //                 new TrapezoidProfile.State(currentPositionRot, currentVelocityRPS);
        //     }
        // }

        inputs.appliedVolts = appliedVolts;
        inputs.currentStatorAmps = 0.0; // Kinematic model doesn't simulate current
        inputs.currentSupplyAmps = 0.0;
        inputs.positionRot = currentPositionRot;
        inputs.velocityRotPerSecond = currentVelocityRPS;
        inputs.motorVolts = appliedVolts;
    }

    @Override
    public void setOpenLoopDutyCycle(double dutyCycle) {
        controlMode = InternalControlMode.VOLTAGE;
        appliedVolts = MathUtil.clamp(dutyCycle * 12.0, -12.0, 12.0);
    }

    @Override
    public void setVoltage(double voltage) {
        controlMode = InternalControlMode.VOLTAGE;
        appliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    }

    @Override
    public void setVelVoltSetpoint(AngularVelocity velocity) {
        controlMode = InternalControlMode.VELOCITY;
        velocitySetpointRPS = velocity.in(RotationsPerSecond);
    }

    @Override
    public void setVelVoltSetpoint(AngularVelocity velocity, Voltage feedForwardVoltage) {
        setVelVoltSetpoint(velocity);
    }

    @Override
    public void setVelTCSetpoint(AngularVelocity velocity, Current feedForwardTorqueCurrent) {
        setVelVoltSetpoint(velocity);
    }

    @Override
    public void setPositionSetpoint(Angle position) {
        controlMode = InternalControlMode.POSITION_CONTROL;
        TrapezoidProfile.Constraints defaultConstraints =
                (this.cfg.profile != null)
                        ? this.cfg.profile
                        : new TrapezoidProfile.Constraints(50.0, 100.0);
        if (!currentConstraints.equals(defaultConstraints)) {
            currentConstraints = defaultConstraints;
            profile = new TrapezoidProfile(currentConstraints);
        }
        goalProfileState = new TrapezoidProfile.State(position.in(Rotations), 0.0);
    }

    @Override
    public void setMotionMagicSetpoint(
            Angle position, double velocity, double acceleration, double jerk) {
        controlMode = InternalControlMode.POSITION_CONTROL;
        // Update profile constraints if they differ from current ones
        if (currentConstraints.maxVelocity != velocity
                || currentConstraints.maxAcceleration != acceleration) {
            currentConstraints = new TrapezoidProfile.Constraints(velocity, acceleration);
            profile = new TrapezoidProfile(currentConstraints);
        }
        goalProfileState = new TrapezoidProfile.State(position.in(Rotations), 0.0);
    }

    @Override
    public void setCurrentPositionAsZero() {
        setCurrentPosition(Radians.of(0));
    }

    @Override
    public void setCurrentPosition(Angle position) {
        currentPositionRot = position.in(Rotations);
        currentProfileState = new TrapezoidProfile.State(currentPositionRot, currentVelocityRPS);
    }

    @Override
    public void setNeutralMode(boolean wantsBreak) {
        // In a kinematic simulation, neutral mode doesn't affect the motion
        // as we assume zero friction and zero momentum when voltage is 0.
    }

    @Override
    public void setEnableSoftLimits(boolean forward, boolean reverse) {
        this.forwardSoftLimitEnabled = forward;
        this.reverseSoftLimitEnabled = reverse;
    }

    @Override
    public void updateGains(Slot0Configs slot0) {
        // Gains are ignored in this simple kinematic simulation as it follows setpoints perfectly.
    }
}
