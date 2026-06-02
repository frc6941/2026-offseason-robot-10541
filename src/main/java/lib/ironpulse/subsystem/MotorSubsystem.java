package lib.ironpulse.subsystem;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * Skeleton base for single-motor mechanisms following the AdvantageKit IO style.
 *
 * <p>- Owns IO and AutoLogged inputs - Reads inputs and logs in periodic() - Provides minimal
 * open-loop helper command
 */
public class MotorSubsystem<T extends MotorInputsAutoLogged, U extends MotorIO>
        extends SubsystemBase {
    protected final U io;
    protected final T inputs;
    protected final SubsystemConfig config;

    protected ControlMode mode = ControlMode.STOP;
    protected double setpoint = 0.0;

    public MotorSubsystem(SubsystemConfig config, T inputs, U io) {
        super(config.name);
        this.config = config;
        this.io = io;
        this.inputs = inputs;
        io.setCurrentPosition(Rotations.zero());
    }

    @Override
    public void periodic() {
        io.readInputs(inputs);
        Logger.processInputs(getName(), inputs);
        logState();
    }

    protected void logState() {
        Logger.recordOutput(config.name + "/mode", mode.name());
        Logger.recordOutput(config.name + "/setPoint", setpoint);
    }

    /** Open-loop duty cycle command,([-1.0, 1.0] where 1.0 = 100%). */
    public Command runDutyCycle(double dutyCycle) {
        return Commands.run(
                () -> {
                    setpoint = MathUtil.clamp(dutyCycle, -1.0, 1.0);
                    io.setOpenLoopDutyCycle(setpoint);
                    mode = ControlMode.DUTY_CYCLE;
                },
                this);
    }

    /** Open-loop duty cycle command,([-1.0, 1.0] where 1.0 = 100%). */
    public Command runDutyCycle(DoubleSupplier dutyCycle) {
        return Commands.run(
                () -> {
                    setpoint = MathUtil.clamp(dutyCycle.getAsDouble(), -1.0, 1.0);
                    io.setOpenLoopDutyCycle(setpoint);
                    mode = ControlMode.DUTY_CYCLE;
                },
                this);
    }

    /** Open-loop duty cycle command, -12-12V. */
    public Command runVoltage(double voltage) {
        return Commands.run(
                () -> {
                    setpoint = MathUtil.clamp(voltage, -12.0, 12.0);
                    io.setVoltage(setpoint);
                    mode = ControlMode.VOLTAGE;
                },
                this);
    }

    /** Open-loop voltage setter, used currently only for sysid, otherwise use runVoltage */
    public void setVoltage(Voltage voltage) {
        setpoint = MathUtil.clamp(voltage.in(Volts), -12.0, 12.0);
        io.setVoltage(setpoint);
        mode = ControlMode.SYSID;
    }

    /** Open-loop duty cycle command, -12-12V. */
    public Command runVoltage(DoubleSupplier voltage) {
        return Commands.run(
                () -> {
                    setpoint = MathUtil.clamp(voltage.getAsDouble(), -12.0, 12.0);
                    io.setVoltage(setpoint);
                    mode = ControlMode.VOLTAGE;
                },
                this);
    }

    /** Stop command */
    public Command runStop() {
        return Commands.run(
                () -> {
                    io.setOpenLoopDutyCycle(0.0);
                    mode = ControlMode.STOP;
                    setpoint = 0.0;
                },
                this);
    }

    /** Set neutral mode command,exits instantly */
    public Command setNeutralMode(boolean wantsBreak) {
        return Commands.runOnce(() -> io.setNeutralMode(wantsBreak));
    }

    /** Set stator + supply current limits command, exits instantly */
    public Command setCurrentLimits(double statorCurrentLimitAmps, double supplyCurrentLimitAmps) {
        return Commands.runOnce(
                () -> io.setCurrentLimits(statorCurrentLimitAmps, supplyCurrentLimitAmps));
    }

    public Current getStatorCurrent() {
        return Amps.of(inputs.currentStatorAmps);
    }

    public Current getSupplyCurrent() {
        return Amps.of(inputs.currentSupplyAmps);
    }

    public Voltage getMotorVoltage() {
        return Volts.of(inputs.motorVolts);
    }
}
