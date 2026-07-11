package frc.robot.subsystems.Hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intaker.IntakerConfig.IntakeMode;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;

public class HopperSubsystem extends VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
    private final IntakerSubsystem intaker;

    public HopperSubsystem(
            IntakerSubsystem intaker,
            SubsystemConfig config,
            MotorInputsAutoLogged inputs,
            MotorIO io) {
        super(config, inputs, io, HopperParamsNT.asVelocityParamSources());
        this.intaker = intaker;
    }

    public Command idle() {
        return runVelTC(RotationsPerSecond.of(HopperConfig.HopperParams.idleRPS));
    }

    public Command feed() {
        return runVelTC(RotationsPerSecond.of(HopperConfig.HopperParams.feedRPS));
    }

    public Command shoot() {
        return runVelTC(RotationsPerSecond.of(HopperConfig.HopperParams.shootRPS));
    }

    public Command reverse() {
        return runVelTC(RotationsPerSecond.of(-HopperConfig.HopperParams.feedRPS));
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
