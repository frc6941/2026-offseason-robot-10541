package frc.robot.subsystems.Hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intaker.IntakerConfig.IntakeMode;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import java.util.function.BooleanSupplier;
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

    /**
     * Shoot-speed while {@code gate} holds true (e.g. shooter up to speed AND heading on target),
     * idle-speed otherwise. Re-evaluates every loop, so it drops back to idle immediately if the
     * chassis swings off target mid-feed instead of latching once a shot starts.
     */
    public Command shootWhile(BooleanSupplier gate) {
        return runVelTC(
                () ->
                        RotationsPerSecond.of(
                                gate.getAsBoolean()
                                        ? HopperConfig.HopperParams.shootRPS
                                        : HopperConfig.HopperParams.idleRPS));
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
