package frc.robot.subsystems.Intaker;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
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
                                roller.runVelTC(() -> RotationsPerSecond.of(rollerTargetRps()))
                                        .until(
                                                () ->
                                                        currentMode
                                                                        != IntakerConfig.IntakeMode
                                                                                .INTAKING
                                                                && currentMode
                                                                        != IntakeMode.MAX_INTAKING
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
                                                                || currentMode
                                                                        == IntakeMode.MAX_INTAKING
                                                                || currentMode == IntakeMode.FEEDING
                                                                || currentMode
                                                                        == IntakeMode
                                                                                .EXTENDED_REVERSE
                                                                || currentMode
                                                                        == IntakeMode
                                                                                .RETRACTED_FEEDING),
                                () ->
                                        currentMode == IntakeMode.INTAKING
                                                || currentMode == IntakeMode.MAX_INTAKING
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

    // Rate-limit state for the RETRACTED_FEEDING (shoot) raise, so followModePivot ramps up gently
    // instead of snapping. Only used in auto — see pivotTargetAngle().
    private double shootRampAngleDeg = 0.0;
    private boolean shootRampActive = false;

    private Angle pivotTargetAngle() {
        if (!pivotZeroed) {
            shootRampActive = false;
            return pivot.getCurrPos();
        }

        // The retracted modes are the shoot pose. In auto the pivot is driven here by
        // followModePivot's
        // position PID, which would SNAP to the angle. To match the teleop
        // raisePivotForShootSlowly()
        // gentle raise, rate-limit the commanded angle toward retractedfeedPosAngle at
        // shootRaiseSpeedDegreesPerSecond instead of jumping. (Teleop never reaches this branch —
        // holdRetractedFeedPosition() suppresses this default command and runs the ramp itself. We
        // can't run that ramp in auto because followModePivot already holds the pivot requirement.)
        if (currentMode == IntakeMode.RETRACTED_FEEDING
                || currentMode == IntakeMode.RETRACTED_SHOOTING) {
            double targetDeg = IntakerPivotParamsNT.retractedfeedPosAngle.getValue();
            if (!shootRampActive) {
                shootRampAngleDeg = pivot.getCurrPos().in(Degrees);
                shootRampActive = true;
            }
            double stepDeg =
                    IntakerPivotParamsNT.shootRaiseSpeedDegreesPerSecond.getValue()
                            * RobotConstants.LOOPER_DT;
            double errDeg = targetDeg - shootRampAngleDeg;
            shootRampAngleDeg += Math.copySign(Math.min(Math.abs(errDeg), stepDeg), errDeg);
            return Degrees.of(shootRampAngleDeg);
        }

        shootRampActive = false;
        return switch (currentMode) {
            case INTAKING, MAX_INTAKING ->
                    Degrees.of(IntakerPivotParamsNT.deployPosAngle.getValue());
            case EXTENDED_IDLE, EXTENDED_REVERSE ->
                    Degrees.of(IntakerPivotParamsNT.deployPosAngle.getValue());
            case RETRACTED -> Degrees.of(IntakerPivotParamsNT.retractPosAngle.getValue());
            case FEEDING -> Degrees.of(IntakerPivotParamsNT.feedPosAngle.getValue());
            default -> Degrees.of(IntakerPivotParamsNT.retractPosAngle.getValue());
        };
    }

    private void setIntakeMode(IntakeMode mode) {
        fallbackMode = mode;

        if (currentMode != IntakeMode.FEEDING
                && currentMode != IntakeMode.EXTENDED_REVERSE
                && currentMode != IntakeMode.RETRACTED_FEEDING
                && currentMode != IntakeMode.RETRACTED_SHOOTING) {
            currentMode = mode;
        }
    }

    private double rollerTargetRps() {
        if (currentMode == IntakeMode.EXTENDED_REVERSE) {
            return IntakerRollerParamsNT.outtakeRPS.getValue();
        }
        if (currentMode == IntakeMode.MAX_INTAKING) {
            return IntakerRollerParamsNT.intakeRPSmax.getValue();
        }
        return IntakerRollerParamsNT.intakeRPS.getValue();
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

    public Command runMaxIntakeContinuous() {
        return Commands.startEnd(
                () -> setIntakeMode(IntakeMode.MAX_INTAKING),
                () -> setIntakeMode(IntakeMode.EXTENDED_IDLE),
                this);
    }

    public Command runRetract() {
        return Commands.runOnce(() -> setIntakeMode(IntakeMode.RETRACTED));
    }

    public Command runExtendedIdle() {
        return Commands.runOnce(() -> setIntakeMode(IntakeMode.EXTENDED_IDLE));
    }

    /**
     * Unconditionally drop to EXTENDED_IDLE (bypassing the setIntakeMode guard that protects
     * FEEDING/EXTENDED_REVERSE/RETRACTED_FEEDING). Used to guarantee the intake — and therefore the
     * hopper, whose default speed is driven by the intake mode — actually stops after an auto shot,
     * regardless of what mode the shot left behind.
     */
    public Command forceExtendedIdle() {
        return Commands.runOnce(
                () -> {
                    fallbackMode = IntakeMode.EXTENDED_IDLE;
                    currentMode = IntakeMode.EXTENDED_IDLE;
                });
    }

    public Command runFeed() {
        return Commands.startEnd(
                () -> currentMode = IntakeMode.FEEDING, () -> currentMode = fallbackMode, this);
    }

    /**
     * Hold the {@link IntakeMode#RETRACTED_FEEDING} mode WITHOUT commanding the pivot directly —
     * the pivot is moved by whatever is reading the mode (in auto, {@link #followModePivot()}).
     * This exists because {@link #holdRetractedFeedPosition()} takes the pivot requirement via
     * {@link #raisePivotForShootSlowly()}, which can't run in auto: {@code followModePivot()}
     * already holds the pivot for the whole routine, and two commands can't require the same
     * subsystem in one composition. Requires only the intake (mode) subsystem, so it's safe to run
     * in parallel with {@code followModePivot()}. Because the pivot is driven by {@code
     * pivotTargetAngle()}'s position PID here (not the {@code raisePivotForShootSlowly()} ramp),
     * the raise is at PID speed, not the gentle teleop ramp.
     */
    public Command holdRetractedFeedMode() {
        return Commands.runEnd(
                () -> currentMode = IntakeMode.RETRACTED_FEEDING,
                () -> {
                    fallbackMode = IntakeMode.EXTENDED_IDLE;
                    currentMode = IntakeMode.EXTENDED_IDLE;
                },
                this);
    }

    /** Hold the shoot pivot pose while keeping the intake roller stopped. */
    public Command holdRetractedShootMode() {
        return Commands.runEnd(
                () -> currentMode = IntakeMode.RETRACTED_SHOOTING,
                () -> {
                    fallbackMode = IntakeMode.EXTENDED_IDLE;
                    currentMode = IntakeMode.EXTENDED_IDLE;
                },
                this);
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

    /** Raise to the shoot pose slowly without running the intake roller. */
    public Command holdRetractedShootPosition() {
        return Commands.parallel(holdRetractedShootMode(), raisePivotForShootSlowly());
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

    public Command raisePivotForShootSlowly() {
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

        return initializeRamp.andThen(followRamp).finallyDo(interrupted -> timer.stop());
    }

    public Command runExtendedReverse() {
        return Commands.parallel(
                Commands.startEnd(
                        () -> currentMode = IntakeMode.EXTENDED_REVERSE,
                        () -> currentMode = fallbackMode,
                        this),
                roller.runVelTC(
                        () -> RotationsPerSecond.of(IntakerRollerParamsNT.outtakeRPS.getValue())));
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
