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

    // Pass suppliers, not snapshot values: the default command's sub-commands are built once at
    // boot (via Commands.select/Map.of), so a captured RotationsPerSecond.of(...) would freeze the
    // RPS to its startup value and ignore live NT edits. A supplier re-reads every loop.
    public Command idle() {
        return runVelTC(() -> RotationsPerSecond.of(HopperParamsNT.idleRPS.getValue()));
    }

    public Command feed() {
        return runVelTC(() -> RotationsPerSecond.of(HopperParamsNT.feedRPS.getValue()));
    }

    public Command shoot() {
        return runVelTC(() -> RotationsPerSecond.of(HopperParamsNT.shootRPS.getValue()));
    }

    public Command reverse() {
        return runVelTC(() -> RotationsPerSecond.of(-HopperParamsNT.feedRPS.getValue()));
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
