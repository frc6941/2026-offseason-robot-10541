package frc.robot.subsystems.Intaker;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intaker.IntakerConfig.IntakeMode;
import frc.robot.subsystems.Intaker.IntakerConfig.IntakerRollerParams;
import frc.robot.subsystems.Intaker.IntakerConfig.IntakerPivotParams;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lombok.Getter;

public class IntakerSubsystem extends SubsystemBase{
    private VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> roller;
    private PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> pivot;

    @Getter
    @AutoLogOutput(key = "Intaker/state")
    private IntakeMode currentMode = IntakeMode.RETRACTED;

    @AutoLogOutput(key = "Intaker/fallbackState")
    private IntakeMode fallbackMode = IntakeMode.RETRACTED;

    public IntakerSubsystem(VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> roller,
                            PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> pivot){
        this.roller = roller;
        this.pivot = pivot;

    }



    public void setDefaultCommand(){
        roller.setDefaultCommand(
            Commands.either(roller.runVelTC(() -> 
                                            RotationsPerSecond.of(
                                                currentMode == IntakeMode.EXTENDED_REVERSE
                                                ? IntakerRollerParamsNT.outtakeRPS.getValue()
                                                : IntakerRollerParamsNT.intakeRPS.getValue()))
                                    .until(() ->
                                            currentMode != IntakerConfig.IntakeMode.INTAKING
                                            && currentMode != IntakeMode.FEEDING
                                            && currentMode != IntakeMode.EXTENDED_REVERSE
                                            && currentMode != IntakeMode.RETRACTED_FEEDING), 
                            roller.runStop()
                                    .until(() -> 
                                            currentMode == IntakeMode.INTAKING
                                            || currentMode == IntakeMode.FEEDING
                                            || currentMode == IntakeMode.EXTENDED_REVERSE
                                            || currentMode == IntakeMode.RETRACTED_FEEDING),
                            () ->
                                currentMode == IntakeMode.INTAKING
                                || currentMode == IntakeMode.FEEDING
                                || currentMode == IntakeMode.EXTENDED_REVERSE
                                || currentMode == IntakeMode.RETRACTED_FEEDING
            ).repeatedly()
                            
        );

        pivot.setDefaultCommand(
            pivot.runMotionMagic(
                    () -> switch (currentMode){
                        case INTAKING -> Degrees.of(
                            IntakerPivotParamsNT.deployPosAngle.getValue()
                        );
                        case EXTENDED_IDLE, EXTENDED_REVERSE -> Degrees.of(
                            IntakerPivotParamsNT.deployPosAngle.getValue()
                        );
                        case RETRACTED -> Degrees.of(
                            IntakerPivotParamsNT.retractPosAngle.getValue()
                        );
                        case FEEDING -> Degrees.of(
                            IntakerPivotParamsNT.feedPosAngle.getValue()
                        );
                        case RETRACTED_FEEDING -> Degrees.of(
                            IntakerPivotParamsNT.retractedfeedPosAngle.getValue()
                        );
                        default -> Degrees.of(
                            IntakerPivotParamsNT.retractPosAngle.getValue()
                        );
                    }
            )
        );
            
    }
    

    public Command runIntake(){
        return Commands.runOnce(
            () -> {
                fallbackMode = IntakeMode.INTAKING;

                if(currentMode != IntakeMode.FEEDING 
                    && currentMode != IntakeMode.EXTENDED_REVERSE){
                    currentMode = IntakeMode.INTAKING;
                }

            }
        );
    }

    public Command runRetract(){
        return Commands.runOnce(
            () -> {
                fallbackMode = IntakeMode.RETRACTED;

                if(currentMode != IntakeMode.FEEDING 
                    && currentMode != IntakeMode.EXTENDED_REVERSE){
                    currentMode = IntakeMode.RETRACTED;
                }

            }
        );
    }

    public Command runExtendedIdle(){
        return Commands.runOnce(
            () -> {
                fallbackMode = IntakeMode.EXTENDED_IDLE;

                if(currentMode != IntakeMode.FEEDING 
                    && currentMode != IntakeMode.EXTENDED_REVERSE){
                    currentMode = IntakeMode.EXTENDED_IDLE;
                }

            }
        );
    }

    public Command runFeed(){
        return Commands.startEnd(
            () -> currentMode = IntakeMode.FEEDING,
            () -> currentMode = fallbackMode
        );
    }

    public Command runExtendedReverse(){
        return Commands.startEnd(
            () -> currentMode = IntakeMode.EXTENDED_REVERSE,
            () -> currentMode = fallbackMode
        );
    }

    public Command zeroCommand(){
        return pivot.zeroCommand();
    }
}
