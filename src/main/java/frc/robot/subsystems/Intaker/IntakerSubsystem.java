package frc.robot.subsystems.Intaker;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
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
            case INTAKING -> Degrees.of(IntakerPivotParamsNT.deployPosAngle.getValue());
            case EXTENDED_IDLE, EXTENDED_REVERSE ->
                    Degrees.of(IntakerPivotParamsNT.deployPosAngle.getValue());
            case RETRACTED -> Degrees.of(IntakerPivotParamsNT.retractPosAngle.getValue());
            case FEEDING -> Degrees.of(IntakerPivotParamsNT.feedPosAngle.getValue());
            case RETRACTED_FEEDING ->
                    Degrees.of(IntakerPivotParamsNT.retractedfeedPosAngle.getValue());
            default -> Degrees.of(IntakerPivotParamsNT.retractPosAngle.getValue());
        };
    }

    private void setIntakeMode(IntakeMode mode) {
        fallbackMode = mode;

        if (currentMode != IntakeMode.FEEDING
                && currentMode != IntakeMode.EXTENDED_REVERSE
                && currentMode != IntakeMode.RETRACTED_FEEDING) {
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
                () -> currentMode = IntakeMode.FEEDING, () -> currentMode = fallbackMode, this);
    }

    public Command holdRetractedFeedPosition() {
        Command holdShootMode =
                Commands.runEnd(
                        () -> currentMode = IntakeMode.RETRACTED_FEEDING,
                        () -> {
                            fallbackMode = IntakeMode.EXTENDED_IDLE;
                            currentMode = IntakeMode.EXTENDED_IDLE;
                        },
                        this);

        return Commands.parallel(holdShootMode, raisePivotForShootSlowly());
    }

    public Command returnPivotToIdleFast() {
        Command selectIdleMode =
                Commands.runOnce(
                        () -> {
                            fallbackMode = IntakeMode.EXTENDED_IDLE;
                            currentMode = IntakeMode.EXTENDED_IDLE;
                        },
                        this);
        Command commandIdlePosition =
                pivot.runPosition(() -> Degrees.of(IntakerPivotParamsNT.deployPosAngle.getValue()))
                        .withTimeout(0.05);

        return selectIdleMode.andThen(commandIdlePosition);
    }

    private Command raisePivotForShootSlowly() {
        Timer timer = new Timer();
        double[] commandedAngleDeg = {0.0};
        double[] lastTimeSeconds = {0.0};

        Command initializeRamp =
                Commands.runOnce(
                        () -> {
                            commandedAngleDeg[0] = pivot.getCurrPos().in(Degrees);
                            lastTimeSeconds[0] = 0.0;
                            timer.restart();
                        },
                        pivot);
        Command followRamp =
                pivot.runPosition(
                        () -> {
                            double nowSeconds = timer.get();
                            double maxStepDeg =
                                    IntakerPivotParamsNT.shootRaiseSpeedDegreesPerSecond.getValue()
                                            * Math.max(0.0, nowSeconds - lastTimeSeconds[0]);
                            lastTimeSeconds[0] = nowSeconds;

                            double targetDeg =
                                    IntakerPivotParamsNT.retractedfeedPosAngle.getValue();
                            double errorDeg = targetDeg - commandedAngleDeg[0];
                            commandedAngleDeg[0] +=
                                    Math.copySign(
                                            Math.min(Math.abs(errorDeg), maxStepDeg), errorDeg);
                            return Degrees.of(commandedAngleDeg[0]);
                        });

        return Commands.waitSeconds(1.6)
                .andThen(initializeRamp.andThen(followRamp).finallyDo(interrupted -> timer.stop()));
    }

    public Command runExtendedReverse() {
        return Commands.startEnd(
                () -> currentMode = IntakeMode.EXTENDED_REVERSE,
                () -> currentMode = fallbackMode,
                this);
    }

    /**
     * Continuously drive the pivot to the current intake mode's target angle. Run this in parallel
     * with an autonomous routine (after homing) so intake-mode changes actuate the pivot even
     * though the pivot's own default command is suppressed while the auto command group holds the
     * pivot requirement (e.g. via {@link #zeroCommand()}). Without it, {@code runIntake()}/{@code
     * runRetract()} only flip {@code currentMode} and never move the pivot during an auto.
     */
    public Command followModePivot() {
        return runPivotTo(this::pivotTargetAngle);
    }

    public Command zeroCommand() {
        return Commands.runOnce(() -> pivotZeroed = false)
                .andThen(pivot.zeroCommand())
                .andThen(Commands.runOnce(this::finishPivotZeroing));
    }

    private void finishPivotZeroing() {
        pivotZeroed = true;
        fallbackMode = IntakeMode.EXTENDED_IDLE;
        currentMode = IntakeMode.EXTENDED_IDLE;
    }

    /** Live pivot angle, for 3D mechanism visualization / logging. */
    public Angle getPivotAngle() {
        return pivot.getCurrPos();
    }
}
