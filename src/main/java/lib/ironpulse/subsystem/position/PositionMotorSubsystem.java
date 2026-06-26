package lib.ironpulse.subsystem.position;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.function.Supplier;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.ControlMode;
import lib.ironpulse.subsystem.MotorSubsystem;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.utils.LoggedTracer;
import org.littletonrobotics.junction.Logger;

/**
 * Generic position-based motor subsystem.
 *
 * @param <T> The type of autologged inputs.
 * @param <U> The type of IO interface.
 * @param <M> The type of measure for the mechanism.
 */
@SuppressWarnings({"rawtypes", "unchecked"})
public class PositionMotorSubsystem<
                T extends MotorInputsAutoLogged, U extends MotorIO, M extends Measure<?>>
        extends MotorSubsystem<T, U> {

    protected final PositionParamSources params;
    private final Angle zeroOffset;
    private final Slot0Configs slot0Configs;
    private final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    private final M mechanismUnitPerRotation;
    private M currSetpoint;
    private LinearFilter currentFilter = LinearFilter.movingAverage(5);
    private double currentFilterValue = 0.0;

    /**
     * Creates a new generic PositionMotorSubsystem.
     *
     * <p>NOTE: This class must be parameterized when used (e.g., {@code PositionMotorSubsystem<...,
     * Angle>}) to avoid type safety warnings
     *
     * @param config The subsystem configuration (IDs, buses, ratios).
     * @param inputs The autologged inputs for the mechanism.
     * @param io The IO interface for the hardware.
     * @param params The parameter sources (PID, MotionMagic limits).
     * @param initialSetpoint The starting position in mechanism units (M).
     * @param mechanismUnitPerRotation The amount of mechanism movement per ONE IO rotation. Example
     *     (Elevator): Meters.of(METERS_PER_ROTATION) Example (Pivot): Degrees.of(360)
     */
    public PositionMotorSubsystem(
            SubsystemConfig config,
            T inputs,
            U io,
            PositionParamSources params,
            M initialSetpoint,
            M mechanismUnitPerRotation) {
        super(config, inputs, io);
        this.params = params;
        this.zeroOffset = config.zeroOffset;
        this.mechanismUnitPerRotation = mechanismUnitPerRotation;
        this.currSetpoint = initialSetpoint;
        this.mode = ControlMode.POSITION;
        slot0Configs = new Slot0Configs();
        slot0Configs.kP = params.kP();
        slot0Configs.kI = params.kI();
        slot0Configs.kD = params.kD();
        slot0Configs.kA = params.kA();
        slot0Configs.kV = params.kV();
        slot0Configs.kS = params.kS();
        slot0Configs.kG = params.kG();
        io.updateGains(slot0Configs);

        io.setPositionSetpoint(toAngle(initialSetpoint).minus(zeroOffset));
    }

    private Angle toAngle(M mechanismValue) {
        Unit mechanismUnit = (Unit) mechanismUnitPerRotation.unit();
        return Rotations.of(
                ((Measure) mechanismValue).in(mechanismUnit)
                        / ((Measure) mechanismUnitPerRotation).in(mechanismUnit));
    }

    private M fromAngle(Angle motorAngle) {
        return (M) mechanismUnitPerRotation.times(motorAngle.in(Rotations));
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
            this.slot0Configs.kG = params.kG();
            io.setNeutralMode(params.isBrake());
            io.updateGains(slot0Configs);
        }

        LoggedTracer.record(config.name);
    }

    @Override
    protected void logState() {
        Logger.recordOutput(config.name + "/mode", mode.name());
        Unit mechanismUnit = (Unit) mechanismUnitPerRotation.unit();
        if (mode == ControlMode.POSITION || mode == ControlMode.MOTION_MAGIC) {
            Logger.recordOutput(
                    config.name + "/setPoint", ((Measure) currSetpoint).in(mechanismUnit));
        } else {
            Logger.recordOutput(config.name + "/setPoint", setpoint);
        }

        Logger.recordOutput(config.name + "/atGoal", positionAtGoal());
        Logger.recordOutput(
                config.name + "/currPosition", ((Measure) getCurrPos()).in(mechanismUnit));
    }

    public boolean positionAtGoal(M tolerance) {
        // Cast to raw Measure to bypass generic issues with isNear in some WPILib versions
        return ((Measure) getCurrPos()).isNear((Measure) currSetpoint, (Measure) tolerance);
    }

    public boolean positionAtGoal() {
        if (mechanismUnitPerRotation instanceof Angle) {
            return positionAtGoal((M) Degrees.of(params.positionAtGoalToleranceDegrees()));
        }
        return positionAtGoal((M) Meters.of(params.positionAtGoalToleranceMeters()));
    }

    public Command waitUntilAtGoal(M tolerance) {
        return Commands.waitUntil(() -> positionAtGoal(tolerance));
    }

    public Command waitUntilAtGoal() {
        return Commands.waitUntil(this::positionAtGoal);
    }

    public Command runMotionMagic(M setPoint) {
        return runMotionMagic(() -> setPoint);
    }

    public Command runMotionMagic(Supplier<M> setPoint) {
        return Commands.run(
                () -> {
                    M sp = setPoint.get();
                    motionMagicConfigs.MotionMagicAcceleration = params.motionMagicAccelRPS2();
                    motionMagicConfigs.MotionMagicCruiseVelocity = params.motionMagicVelRPS();
                    motionMagicConfigs.MotionMagicJerk = params.motionMagicJerkRPS3();
                    io.setMotionMagicSetpoint(
                            toAngle(sp).minus(zeroOffset),
                            motionMagicConfigs.MotionMagicCruiseVelocity,
                            motionMagicConfigs.MotionMagicAcceleration,
                            motionMagicConfigs.MotionMagicJerk);
                    currSetpoint = sp;
                    mode = ControlMode.MOTION_MAGIC;
                },
                this);
    }

    public Command runMotionMagic(M setPoint, double velocity, double acceleration, double jerk) {
        return runMotionMagic(() -> setPoint, velocity, acceleration, jerk);
    }

    public Command runMotionMagic(
            Supplier<M> setPoint, double velocity, double acceleration, double jerk) {
        return Commands.run(
                () -> {
                    M sp = setPoint.get();
                    motionMagicConfigs.MotionMagicAcceleration = acceleration;
                    motionMagicConfigs.MotionMagicCruiseVelocity = velocity;
                    motionMagicConfigs.MotionMagicJerk = jerk;
                    io.setMotionMagicSetpoint(
                            toAngle(sp).minus(zeroOffset),
                            motionMagicConfigs.MotionMagicCruiseVelocity,
                            motionMagicConfigs.MotionMagicAcceleration,
                            motionMagicConfigs.MotionMagicJerk);
                    currSetpoint = sp;
                    mode = ControlMode.MOTION_MAGIC;
                },
                this);
    }

    public Command runPosition(M setPoint) {
        return runPosition(() -> setPoint);
    }

    public Command runPosition(Supplier<M> setPoint) {
        return Commands.run(
                () -> {
                    M sp = setPoint.get();
                    io.setPositionSetpoint(toAngle(sp).minus(zeroOffset));
                    currSetpoint = sp;
                    mode = ControlMode.POSITION;
                },
                this);
    }

    /**
     * Returns a command that zeroes the mechanism by driving it until a current spike is detected.
     *
     * @return The zeroing command.
     */
    public Command zeroCommand() {
        double zeroVoltage = MathUtil.clamp(config.zeroingConfig.zeroingVoltage, -12.0d, 12.0d);

        Command realZero =
                Commands.runOnce(
                                () -> {
                                    io.setEnableSoftLimits(false, false);
                                    currentFilter =
                                            LinearFilter.movingAverage(
                                                    config.zeroingConfig.zeroingFilterSize);
                                    currentFilterValue = 0.0;
                                },
                                this)
                        .andThen(
                                Commands.deadline(
                                        Commands.run(
                                                        () ->
                                                                currentFilterValue =
                                                                        currentFilter.calculate(
                                                                                getStatorCurrent()
                                                                                        .in(Amps)))
                                                .until(
                                                        () ->
                                                                currentFilterValue
                                                                        > config.zeroingConfig
                                                                                .zeroingCurrentLimit),
                                        runVoltage(() -> zeroVoltage)))
                        .andThen(Commands.runOnce(io::setCurrentPositionAsZero, this))
                        .finallyDo(
                                () -> {
                                    mode = ControlMode.VOLTAGE;
                                    setpoint = 0.0;
                                    io.setVoltage(0.0);
                                    io.setEnableSoftLimits(true, true);
                                });

        Command simZero = Commands.runOnce(io::setCurrentPositionAsZero, this);

        return new ConditionalCommand(realZero, simZero, RobotBase::isReal);
    }

    public M getCurrPos() {
        return fromAngle(Rotations.of(inputs.positionRot + zeroOffset.in(Rotations)));
    }

    public void setCurrPos(M pos) {
        io.setCurrentPosition(toAngle(pos).minus(zeroOffset));
    }

    public void setEnableSoftLimits(boolean forward, boolean reverse) {
        io.setEnableSoftLimits(forward, reverse);
    }

    public M getCurrSetpoint() {
        return currSetpoint;
    }
}
