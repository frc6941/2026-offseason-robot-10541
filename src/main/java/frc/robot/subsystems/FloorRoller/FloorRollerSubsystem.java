package frc.robot.subsystems.FloorRoller;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intaker.IntakerConfig.IntakeMode;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;

public class FloorRollerSubsystem extends VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
    private final IntakerSubsystem intaker;

    public FloorRollerSubsystem(
            IntakerSubsystem intaker,
            SubsystemConfig config,
            MotorInputsAutoLogged inputs,
            MotorIO io) {
        super(config, inputs, io, FloorRollerParamsNT.asVelocityParamSources());
        this.intaker = intaker;
    }

    public Command idle() {
        return runVelTC(RotationsPerSecond.of(FloorRollerParamsNT.idleRPS.getValue()));
    }

    public Command feed() {
        return runVelTC(RotationsPerSecond.of(FloorRollerParamsNT.feedRPS.getValue()));
    }

    public Command shoot() {
        return runVelTC(RotationsPerSecond.of(FloorRollerParamsNT.shootRPS.getValue()));
    }

    public Command reverse() {
        return runVelTC(RotationsPerSecond.of(-FloorRollerParamsNT.feedRPS.getValue()));
    }

    public void configureDefaultCommand() {
        setDefaultCommand(
                Commands.select(
                        java.util.Map.of(
                                IntakeMode.INTAKING, feed(),
                                IntakeMode.FEEDING, feed(),
                                IntakeMode.RETRACTED_FEEDING, reverse(),
                                IntakeMode.EXTENDED_REVERSE, reverse(),
                                IntakeMode.EXTENDED_IDLE, idle(),
                                IntakeMode.RETRACTED, idle()),
                        intaker::getCurrentMode));
    }
}
