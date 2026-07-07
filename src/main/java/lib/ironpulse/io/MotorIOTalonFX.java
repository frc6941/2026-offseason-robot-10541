package lib.ironpulse.io;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
    // FOC stays enabled (Pro-licensed). We use regular Motion Magic instead of Dynamic Motion
    // Magic because the latter needs a CANivore timebase, which these RIO-bus mechanisms lack.
    private final MotionMagicVoltage motionMagicCtrl =
            new MotionMagicVoltage(0.0).withEnableFOC(true);
    private final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    private double lastMMVelocity = Double.NaN;
    private double lastMMAcceleration = Double.NaN;
    private double lastMMJerk = Double.NaN;
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
            // We never read followers back through status signals, so silence their default frames
            // to keep them off the (shared roboRIO) CAN bus. The Follower control request still
            // drives them; only telemetry publishing is reduced.
            followers[i].optimizeBusUtilization();
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
        // Configure update frequencies and register signals.
        // NOTE: these mechanisms live on the roboRIO 1 Mbps CAN bus (only swerve is on the CANivore).
        // 1000 Hz pos/vel per device saturates that shared bus (~11 devices -> 100% utilization,
        // devices dropping offline). The TalonFX closed loop runs internally at 1 kHz regardless of
        // these rates — this only sets how often the RIO reads back for logging/atGoal — so 100 Hz
        // is ample. If a device is ever moved back onto the CANivore, these can be raised again.
        posSig.setUpdateFrequency(100.0);
        velSig.setUpdateFrequency(100.0);
        motorVoltSig.setUpdateFrequency(50.0);
        supplyVoltSig.setUpdateFrequency(10.0);
        statorSig.setUpdateFrequency(50.0);
        supplySig.setUpdateFrequency(20.0);
        PhoenixUtils.registerSignals(cfg.mainBus, signals);
        main.optimizeBusUtilization();
    }

    private void configureCANcoder(SubsystemConfig.RemoteCANcoder rc) {
        CANcoder coder = new CANcoder(rc.id, rc.bus);
        // Build a CANcoderConfiguration from rc fields
        CANcoderConfiguration c = new CANcoderConfiguration();
        c.MagnetSensor.MagnetOffset = rc.magnetOffset;
        c.MagnetSensor.SensorDirection = rc.sensorDirection;
        coder.getConfigurator().apply(c);
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
        // Regular Motion Magic reads its profile constraints from the device's MotionMagicConfigs
        // (only Dynamic Motion Magic takes them per-request). Re-apply only when they change to
        // avoid a blocking config write every loop; in practice these are constant per subsystem,
        // so this applies once.
        if (velocity != lastMMVelocity
                || acceleration != lastMMAcceleration
                || jerk != lastMMJerk) {
            motionMagicConfigs.MotionMagicCruiseVelocity = velocity;
            motionMagicConfigs.MotionMagicAcceleration = acceleration;
            motionMagicConfigs.MotionMagicJerk = jerk;
            PhoenixUtils.tryUntilOk(5, () -> main.getConfigurator().apply(motionMagicConfigs));
            lastMMVelocity = velocity;
            lastMMAcceleration = acceleration;
            lastMMJerk = jerk;
        }
        main.setControl(motionMagicCtrl.withPosition(position));
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
