package lib.ironpulse.subsystem.velocity;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.ControlMode;
import lib.ironpulse.subsystem.MotorSubsystem;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.utils.LoggedTracer;
import org.littletonrobotics.junction.Logger;

/**
 * Velocity subsystem for velocity-controlled mechanisms (shooters, intakes, etc.) Extends {@link
 * MotorSubsystem} with velocity control, velocity-at-goal checking, and tunable PID.
 */
public class VelocityMotorSubsystem<T extends MotorInputsAutoLogged, U extends MotorIO>
        extends MotorSubsystem<T, U> {

    private AngularVelocity currSetpoint = RotationsPerSecond.of(0.0);

    protected final VelocityParamSources params;
    private final Slot0Configs slot0Configs;

    public VelocityMotorSubsystem(
            SubsystemConfig config, T inputs, U io, VelocityParamSources params) {
        super(config, inputs, io);
        this.params = params;
        slot0Configs = new Slot0Configs();
        slot0Configs.kP = params.kP();
        slot0Configs.kI = params.kI();
        slot0Configs.kD = params.kD();
        slot0Configs.kA = params.kA();
        slot0Configs.kV = params.kV();
        slot0Configs.kS = params.kS();
        io.updateGains(slot0Configs);
    }

    @Override
    public void periodic() {
        super.periodic();
        if (params.hasChanged()) {
            this.slot0Configs.kP = params.kP();
            this.slot0Configs.kI = params.kI();
            this.slot0Configs.kD = params.kD();
            this.slot0Configs.kA = params.kA();
            this.slot0Configs.kV = params.kV();
            this.slot0Configs.kS = params.kS();
            io.setNeutralMode(params.isBrake());
            io.updateGains(slot0Configs);
        }

        LoggedTracer.record(config.name);
    }

    @Override
    protected void logState() {
        Logger.recordOutput(config.name + "/mode", mode.name());
        if (mode == ControlMode.VEL_VOL || mode == ControlMode.VEL_TC) {
            Logger.recordOutput(config.name + "/setPoint", currSetpoint.in(RotationsPerSecond));
        } else {
            Logger.recordOutput(config.name + "/setPoint", setpoint);
        }

        Logger.recordOutput(config.name + "/atGoal", velocityAtGoal());
        Logger.recordOutput(config.name + "/currVelocity", getVelocity().in(RotationsPerSecond));
    }

    /** Set voltage-domain velocity setpoint command. */
    public Command runVelVolt(AngularVelocity velocity) {
        return runVelVolt(() -> velocity);
    }

    /** Set voltage-domain velocity setpoint command. */
    public Command runVelVolt(Supplier<AngularVelocity> velocity) {
        return Commands.run(
                () -> {
                    AngularVelocity sp = velocity.get();
                    io.setVelVoltSetpoint(sp);
                    currSetpoint = sp;
                    mode = ControlMode.VEL_VOL;
                },
                this);
    }

    /** Set voltage-domain velocity setpoint command with voltage feedforward. */
    public Command runVelVolt(Supplier<AngularVelocity> velocity, Supplier<Voltage> feedForward) {
        return Commands.run(
                () -> {
                    AngularVelocity sp = velocity.get();
                    io.setVelVoltSetpoint(sp, feedForward.get());
                    currSetpoint = sp;
                    mode = ControlMode.VEL_VOL;
                },
                this);
    }

    /** Set voltage-domain velocity setpoint command with voltage feedforward. */
    public Command runVelVolt(AngularVelocity velocity, Voltage feedForward) {
        return runVelVolt(() -> velocity, () -> feedForward);
    }

    /** Set torque-current FOC velocity setpoint command with zero additional feedforward. */
    public Command runVelTC(Supplier<AngularVelocity> velocity) {
        return runVelTC(velocity, () -> Amps.of(0.0));
    }

    /** Set torque-current FOC velocity setpoint command with zero additional feedforward. */
    public Command runVelTC(AngularVelocity velocity) {
        return runVelTC(() -> velocity, () -> Amps.of(0.0));
    }

    /** Set torque-current FOC velocity setpoint command with torque-current feedforward. */
    public Command runVelTC(Supplier<AngularVelocity> velocity, Supplier<Current> feedForward) {
        return Commands.run(
                () -> {
                    AngularVelocity sp = velocity.get();
                    io.setVelTCSetpoint(sp, feedForward.get());
                    currSetpoint = sp;
                    mode = ControlMode.VEL_TC;
                },
                this);
    }

    /** Set torque-current FOC velocity setpoint command with torque-current feedforward. */
    public Command runVelTC(AngularVelocity velocity, Current feedForward) {
        return runVelTC(() -> velocity, () -> feedForward);
    }

    /** Check if velocity is within tolerance of setpoint. */
    public boolean velocityAtGoal(AngularVelocity tolerance) {
        return getVelocity().isNear(currSetpoint, tolerance);
    }

    /** Check if velocity is within default tolerance of setpoint. */
    public boolean velocityAtGoal() {
        return velocityAtGoal(RotationsPerSecond.of(params.velocityAtGoalToleranceRPS()));
    }

    public Command waitUntilAtGoal(AngularVelocity tolerance) {
        return Commands.waitUntil(() -> velocityAtGoal(tolerance));
    }

    public Command waitUntilAtGoal() {
        return Commands.waitUntil(this::velocityAtGoal);
    }

    /** Get current velocity in rotations per second. */
    public AngularVelocity getVelocity() {
        return RotationsPerSecond.of(inputs.velocityRotPerSecond);
    }

    public Angle getPosition() {
        return Rotations.of(inputs.positionRot);
    }

    /** Check if motor is connected. */
    public boolean isConnected() {
        return io.isConnected();
    }

    public AngularVelocity getCurrSetpoint() {
        return currSetpoint;
    }
}
