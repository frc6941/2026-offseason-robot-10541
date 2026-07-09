package frc.robot.subsystems.Intaker;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intaker.IntakerConfig.IntakeMode;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class IntakerSubsystem extends SubsystemBase {
    private VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> roller;
    private PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> pivot;

    @Getter
    @AutoLogOutput(key = "Intaker/state")
    private IntakeMode currentMode = IntakeMode.RETRACTED;

    @AutoLogOutput(key = "Intaker/fallbackState")
    private IntakeMode fallbackMode = IntakeMode.RETRACTED;

    @AutoLogOutput(key = "Intaker/pivotZeroed")
    private boolean pivotZeroed = false;

    public IntakerSubsystem(
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> roller,
            PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> pivot) {
        this.roller = roller;
        this.pivot = pivot;
    }

    public void setDefaultCommand() {
        roller.setDefaultCommand(
                Commands.either(
                                roller.runVelTC(
                                                () ->
                                                        RotationsPerSecond.of(
                                                                currentMode
                                                                                == IntakeMode
                                                                                        .EXTENDED_REVERSE
                                                                        ? IntakerRollerParamsNT
                                                                                .outtakeRPS
                                                                                .getValue()
                                                                        : IntakerRollerParamsNT
                                                                                .intakeRPS
                                                                                .getValue()))
                                        .until(
                                                () ->
                                                        currentMode
                                                                        != IntakerConfig.IntakeMode
                                                                                .INTAKING
                                                                && currentMode != IntakeMode.FEEDING
                                                                && currentMode
                                                                        != IntakeMode
                                                                                .EXTENDED_REVERSE
                                                                && currentMode
                                                                        != IntakeMode
                                                                                .RETRACTED_FEEDING),
                                roller.runStop()
                                        .until(
                                                () ->
                                                        currentMode == IntakeMode.INTAKING
                                                                || currentMode == IntakeMode.FEEDING
                                                                || currentMode
                                                                        == IntakeMode
                                                                                .EXTENDED_REVERSE
                                                                || currentMode
                                                                        == IntakeMode
                                                                                .RETRACTED_FEEDING),
                                () ->
                                        currentMode == IntakeMode.INTAKING
                                                || currentMode == IntakeMode.FEEDING
                                                || currentMode == IntakeMode.EXTENDED_REVERSE
                                                || currentMode == IntakeMode.RETRACTED_FEEDING)
                        .repeatedly());

        pivot.setDefaultCommand(
                pivot.runStop()
                        .until(() -> pivotZeroed)
                        .andThen(runPivotTo(this::pivotTargetAngle)));
    }

    private Command runPivotTo(java.util.function.Supplier<Angle> target) {
        return pivot.runPosition(target);
    }

    private Angle pivotTargetAngle() {
        if (!pivotZeroed) {
            return pivot.getCurrPos();
        }

        return switch (currentMode) {
            case INTAKING -> Degrees.of(IntakerConfig.IntakerPivotParams.deployPosAngle);
            case EXTENDED_IDLE, EXTENDED_REVERSE ->
                    Degrees.of(IntakerConfig.IntakerPivotParams.deployPosAngle);
            case RETRACTED -> Degrees.of(IntakerConfig.IntakerPivotParams.retractPosAngle);
            case FEEDING -> Degrees.of(IntakerConfig.IntakerPivotParams.feedPosAngle);
            case RETRACTED_FEEDING ->
                    Degrees.of(IntakerConfig.IntakerPivotParams.retractedfeedPosAngle);
            default -> Degrees.of(IntakerConfig.IntakerPivotParams.retractPosAngle);
        };
    }

    private void setIntakeMode(IntakeMode mode) {
        fallbackMode = mode;

        if (currentMode != IntakeMode.FEEDING && currentMode != IntakeMode.EXTENDED_REVERSE) {
            currentMode = mode;
        }
    }

    public Command runIntake() {
        return Commands.runOnce(() -> setIntakeMode(IntakeMode.INTAKING));
    }

    public Command runIntakeContinuous() {
        return Commands.startEnd(
                () -> setIntakeMode(IntakeMode.INTAKING),
                () -> setIntakeMode(IntakeMode.EXTENDED_IDLE),
                this);
    }

    public Command runRetract() {
        return Commands.runOnce(() -> setIntakeMode(IntakeMode.RETRACTED));
    }

    public Command runExtendedIdle() {
        return Commands.runOnce(
                () -> {
                    fallbackMode = IntakeMode.EXTENDED_IDLE;

                    if (currentMode != IntakeMode.FEEDING
                            && currentMode != IntakeMode.EXTENDED_REVERSE) {
                        currentMode = IntakeMode.EXTENDED_IDLE;
                    }
                });
    }

    public Command runFeed() {
        return Commands.startEnd(
                () -> currentMode = IntakeMode.FEEDING, () -> currentMode = fallbackMode);
    }

    public Command holdRetractedFeedPosition() {
        return Commands.runEnd(
                () -> currentMode = IntakeMode.RETRACTED_FEEDING,
                () -> {
                    fallbackMode = IntakeMode.EXTENDED_IDLE;
                    currentMode = IntakeMode.EXTENDED_IDLE;
                },
                this);
    }

    public Command runExtendedReverse() {
        return Commands.startEnd(
                () -> currentMode = IntakeMode.EXTENDED_REVERSE, () -> currentMode = fallbackMode);
    }

    public Command zeroCommand() {
        return pivot.zeroCommand()
                .andThen(
                        Commands.runOnce(
                                () -> {
                                    pivotZeroed = true;
                                    fallbackMode = IntakeMode.EXTENDED_IDLE;
                                    currentMode = IntakeMode.EXTENDED_IDLE;
                                },
                                this));
    }

    /** Live pivot angle, for 3D mechanism visualization / logging. */
    public Angle getPivotAngle() {
        return pivot.getCurrPos();
    }
}
