package frc.robot.subsystems.Intaker;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intaker.IntakerConfig.IntakeMode;
import java.util.function.Supplier;
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
                                        .until(() -> !isActive()),
                                roller.runStop().until(this::isActive),
                                this::isActive)
                        .repeatedly());

        pivot.setDefaultCommand(
                pivot.runStop().until(() -> pivotZeroed).andThen(runPivotTo(pivotTargetAngle())));
    }

    /**
     * True in the "active" intake modes — deployed and running the roller (intaking, feeding,
     * reversing, or retracted-feeding). Drives the roller default command and is the single source
     * for "is the intake doing something" used by the indicator and the FieldCore telemetry bridge,
     * so that four-way check lives in exactly one place.
     */
    public boolean isActive() {
        return currentMode == IntakeMode.INTAKING
                || currentMode == IntakeMode.FEEDING
                || currentMode == IntakeMode.EXTENDED_REVERSE
                || currentMode == IntakeMode.RETRACTED_FEEDING;
    }

    private Command runPivotTo(Supplier<Angle> target) {
        return pivot.runPosition(target);
    }

    private Supplier<Angle> pivotTargetAngle() {
        // Return ONE supplier that re-reads pivotZeroed/currentMode every loop. runPosition() calls
        // get() each iteration, so the pivot default command tracks live mode changes instead of
        // latching the branch chosen at command-construction time (when pivotZeroed was still
        // false).
        return () -> {
            if (!pivotZeroed) {
                return pivot.getCurrPos();
            }

            return switch (currentMode) {
                case INTAKING -> Degrees.of(IntakerPivotParamsNT.deployPosAngle.getValue());
                case EXTENDED_IDLE, EXTENDED_REVERSE ->
                        Degrees.of(IntakerPivotParamsNT.deployPosAngle.getValue());
                case RETRACTED -> Degrees.of(IntakerPivotParamsNT.retractPosAngle.getValue());
                case FEEDING -> Degrees.of(IntakerPivotParamsNT.feedPosAngle.getValue());
                case RETRACTED_FEEDING ->
                        Degrees.of(IntakerPivotParamsNT.retractedfeedPosAngle.getValue());
                default -> Degrees.of(IntakerPivotParamsNT.retractPosAngle.getValue());
            };
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
        return Commands.runOnce(() -> setIntakeMode(IntakeMode.EXTENDED_IDLE));
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
