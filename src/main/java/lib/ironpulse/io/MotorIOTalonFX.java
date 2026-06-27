package lib.ironpulse.io;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.utils.PhoenixUtils;

/**
 * TalonFX implementation of MotorIO, with optional remote CANcoder feedback and followers.
 *
 * <p>Note: Mechanism units returned by getPosition()/getVelocity() depend on Phoenix Feedback
 * ratios (SensorToMechanismRatio, RotorToSensorRatio) configured via SubsystemConfig.
 */
public class MotorIOTalonFX implements MotorIO {
    private final TalonFX main;
    private final TalonFX[] followers;

    private final PositionVoltage positionCtrl = new PositionVoltage(0.0).withEnableFOC(true);
    private final DynamicMotionMagicVoltage dynamicMotionMagicCtrl =
            new DynamicMotionMagicVoltage(0.0, 0.0, 0.0).withEnableFOC(true);
    private final VelocityVoltage velocityCtrl = new VelocityVoltage(0.0).withEnableFOC(true);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentCtrl =
            new VelocityTorqueCurrentFOC(0.0);
    private final DutyCycleOut dutyCtrl = new DutyCycleOut(0.0).withEnableFOC(true);

    private final StatusSignal<Angle> posSig;
    private final StatusSignal<AngularVelocity> velSig;
    private final StatusSignal<Voltage> motorVoltSig;
    private final StatusSignal<Voltage> supplyVoltSig;
    private final StatusSignal<Current> statorSig;
    private final StatusSignal<Current> supplySig;
    private final BaseStatusSignal[] signals;
    private final TalonFXConfiguration fx;
    private final SubsystemConfig config;
    private boolean connected = false;

    public MotorIOTalonFX(SubsystemConfig cfg) {
        this.main = new TalonFX(cfg.mainId, cfg.mainBus);
        this.config = cfg;

        this.fx = cfg.fxConfig;

        fx.MotorOutput.Inverted = cfg.motorInvertedValue;
        fx.MotorOutput.NeutralMode =
                cfg.defaultBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        if (!Double.isNaN(cfg.statorCurrentLimitAmps)) {
            fx.CurrentLimits.StatorCurrentLimitEnable = true;
            fx.CurrentLimits.StatorCurrentLimit = cfg.statorCurrentLimitAmps;
        } else {
            fx.CurrentLimits.StatorCurrentLimitEnable = false;
        }
        if (!Double.isNaN(cfg.supplyCurrentLimitAmps)) {
            fx.CurrentLimits.SupplyCurrentLimitEnable = true;
            fx.CurrentLimits.SupplyCurrentLimit = cfg.supplyCurrentLimitAmps;
        } else {
            fx.CurrentLimits.SupplyCurrentLimitEnable = false;
        }
        if (!Double.isNaN(cfg.ramp)) {
            fx.withClosedLoopRamps(
                    new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(cfg.ramp));
        }
        // Optional: remote CANcoder feedback configuration
        if (cfg.enableRemoteCANcoder && cfg.remoteCANcoder != null) {
            configureCANcoder(cfg.remoteCANcoder);
            // Bind motor feedback to remote CANcoder
            fx.Feedback.FeedbackSensorSource = cfg.remoteCANcoder.feedbackSensorSource;
            fx.Feedback.FeedbackRemoteSensorID = cfg.remoteCANcoder.id;
            fx.Feedback.RotorToSensorRatio = cfg.remoteCANcoder.rotorToSensorRatio;
            fx.ClosedLoopGeneral.ContinuousWrap = cfg.remoteCANcoder.useContinousWrap;
        }

        // Soft limit enables per config (thresholds should be in fxConfig)
        if (!Double.isNaN(cfg.forwardSoftLimitDegrees.magnitude())) {
            fx.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            fx.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                    cfg.forwardSoftLimitDegrees.in(Rotations);
        } else {
            fx.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        }
        if (!Double.isNaN(cfg.reverseSoftLimitDegrees.magnitude())) {
            fx.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            fx.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                    cfg.reverseSoftLimitDegrees.in(Rotations);
        } else {
            fx.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        }

        fx.Feedback.SensorToMechanismRatio = cfg.SensorToMechanismRatio;

        fx.Slot0.GravityType = cfg.gravityType;
        fx.Slot0.StaticFeedforwardSign = cfg.kSValue;

        // fx.withTorqueCurrent(new TorqueCurrentConfigs().withPeakForwardTorqueCurrent(638.1));

        PhoenixUtils.tryUntilOk(5, () -> main.getConfigurator().apply(fx));

        // Followers
        this.followers = new TalonFX[cfg.followers.length];
        for (int i = 0; i < followers.length; i++) {
            var f = cfg.followers[i];
            followers[i] = new TalonFX(f.id, f.bus);
            followers[i].setControl(new Follower(cfg.mainId, f.opposeMain));
            if (!Double.isNaN(f.statorCurrentLimitAmps)
                    || !Double.isNaN(f.supplyCurrentLimitAmps)) {
                var followerCurrentLimits = new CurrentLimitsConfigs();
                if (!Double.isNaN(f.statorCurrentLimitAmps)) {
                    followerCurrentLimits.StatorCurrentLimitEnable = true;
                    followerCurrentLimits.StatorCurrentLimit = f.statorCurrentLimitAmps;
                }
                if (!Double.isNaN(f.supplyCurrentLimitAmps)) {
                    followerCurrentLimits.SupplyCurrentLimitEnable = true;
                    followerCurrentLimits.SupplyCurrentLimit = f.supplyCurrentLimitAmps;
                }
                followers[i].getConfigurator().apply(followerCurrentLimits);
            }
            if (!Double.isNaN(f.ramp)) {
                followers[i]
                        .getConfigurator()
                        .apply(
                                new ClosedLoopRampsConfigs()
                                        .withVoltageClosedLoopRampPeriod(f.ramp));
            }
        }

        // Signals
        posSig = main.getPosition();
        velSig = main.getVelocity();
        motorVoltSig = main.getMotorVoltage();
        supplyVoltSig = main.getSupplyVoltage();
        statorSig = main.getStatorCurrent();
        supplySig = main.getSupplyCurrent();

        signals =
                new BaseStatusSignal[] {
                    posSig, velSig, motorVoltSig, supplyVoltSig, statorSig, supplySig
                };
        // configure update frequencies and register signals
        posSig.setUpdateFrequency(1000.0);
        velSig.setUpdateFrequency(1000.0);
        motorVoltSig.setUpdateFrequency(100.0);
        supplyVoltSig.setUpdateFrequency(30.0);
        statorSig.setUpdateFrequency(100.0);
        supplySig.setUpdateFrequency(100.0);
        boolean isCanivoreBus = cfg.mainBus == RobotConstants.CANIVORE_CAN_BUS;
        PhoenixUtils.registerSignals(isCanivoreBus, signals);
        main.optimizeBusUtilization();
    }

    private void configureCANcoder(SubsystemConfig.RemoteCANcoder rc) {
        // Create a temporary CANcoder handle to push configuration to the device.
        // Configuration is persisted in CANcoder hardware — the handle is discarded
        // because no runtime interaction with the CANcoder is needed after config.
        try (CANcoder coder = new CANcoder(rc.id, rc.bus)) {
            CANcoderConfiguration c = new CANcoderConfiguration();
            c.MagnetSensor.MagnetOffset = rc.magnetOffset;
            c.MagnetSensor.SensorDirection = rc.sensorDirection;
            coder.getConfigurator().apply(c);
        }
    }

    @Override
    public void readInputs(MotorInputs inputs) {
        connected =
                BaseStatusSignal.isAllGood(
                        posSig, velSig, motorVoltSig, supplyVoltSig, statorSig, supplySig);
        inputs.positionRot = posSig.getValueAsDouble();
        inputs.velocityRotPerSecond = velSig.getValueAsDouble();
        inputs.motorVolts = motorVoltSig.getValueAsDouble();
        inputs.appliedVolts = supplyVoltSig.getValueAsDouble();
        inputs.currentStatorAmps = statorSig.getValueAsDouble();
        inputs.currentSupplyAmps = supplySig.getValueAsDouble();
    }

    @Override
    /** Whether all primary signals are reporting without errors. */
    public boolean isConnected() {
        return connected;
    }

    @Override
    public void setOpenLoopDutyCycle(double dutyCycle) {
        main.setControl(dutyCtrl.withOutput(dutyCycle));
    }

    @Override
    public void setPositionSetpoint(Angle position) {
        main.setControl(positionCtrl.withPosition(position));
    }

    @Override
    public void setMotionMagicSetpoint(
            Angle position, double velocity, double acceleration, double jerk) {
        dynamicMotionMagicCtrl.Velocity = velocity;
        dynamicMotionMagicCtrl.Acceleration = acceleration;
        dynamicMotionMagicCtrl.Jerk = jerk;
        main.setControl(dynamicMotionMagicCtrl.withPosition(position));
    }

    @Override
    public void setVoltage(double voltage) {
        main.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setNeutralMode(boolean wantsBreak) {
        this.fx.MotorOutput.NeutralMode =
                wantsBreak ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        main.getConfigurator().apply(this.fx);
        for (int i = 0; i < followers.length; i++) {
            followers[i].getConfigurator().apply(this.fx);
        }
    }

    @Override
    public void setVelVoltSetpoint(AngularVelocity velocity) {
        main.setControl(velocityCtrl.withVelocity(velocity));
    }

    @Override
    public void setVelVoltSetpoint(AngularVelocity velocity, Voltage feedForwardVoltage) {
        main.setControl(velocityCtrl.withVelocity(velocity).withFeedForward(feedForwardVoltage));
    }

    @Override
    public void setVelTCSetpoint(AngularVelocity velocity, Current feedForwardTorqueCurrent) {
        main.setControl(
                velocityTorqueCurrentCtrl
                        .withVelocity(velocity)
                        .withFeedForward(feedForwardTorqueCurrent));
    }

    @Override
    public void setCurrentPositionAsZero() {
        setCurrentPosition(Units.Rotations.of(0.0));
    }

    @Override
    public void setCurrentPosition(Angle positionRad) {
        main.setPosition(positionRad);
    }

    @Override
    public void setEnableSoftLimits(boolean forward, boolean reverse) {
        if (!Double.isNaN(config.forwardSoftLimitDegrees.magnitude())) {
            fx.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
        } else {
            fx.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        }
        if (!Double.isNaN(config.reverseSoftLimitDegrees.magnitude())) {
            fx.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
        } else {
            fx.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        }
        main.getConfigurator().apply(this.fx);
    }

    @Override
    public void setCurrentLimits(double statorCurrentLimitAmps, double supplyCurrentLimitAmps) {
        if (!Double.isNaN(statorCurrentLimitAmps)) {
            this.fx.CurrentLimits.StatorCurrentLimitEnable = true;
            this.fx.CurrentLimits.StatorCurrentLimit = statorCurrentLimitAmps;
        } else {
            this.fx.CurrentLimits.StatorCurrentLimitEnable = false;
        }
        if (!Double.isNaN(supplyCurrentLimitAmps)) {
            this.fx.CurrentLimits.SupplyCurrentLimitEnable = true;
            this.fx.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimitAmps;
        } else {
            this.fx.CurrentLimits.SupplyCurrentLimitEnable = false;
        }
        main.getConfigurator().apply(this.fx);
    }

    @Override
    public void updateGains(Slot0Configs slot0) {
        this.fx.Slot0 = slot0;
        fx.withSlot0(slot0);
        slot0.GravityType = config.gravityType;
        slot0.StaticFeedforwardSign = config.kSValue;
        main.getConfigurator().apply(this.fx);
    }
}
