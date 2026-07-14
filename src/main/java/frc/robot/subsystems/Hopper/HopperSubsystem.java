package frc.robot.subsystems.Hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
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
                                        ? HopperParamsNT.shootRPS.getValue()
                                        : HopperParamsNT.idleRPS.getValue()));
    }

    public void configureDefaultCommand() {
        setDefaultCommand(runVelTC(() -> RotationsPerSecond.of(defaultTargetRps())));
    }

    private double defaultTargetRps() {
        return switch (intaker.getCurrentMode()) {
            case INTAKING, MAX_INTAKING, FEEDING -> HopperParamsNT.feedRPS.getValue();
            case RETRACTED_FEEDING, EXTENDED_REVERSE -> -HopperParamsNT.feedRPS.getValue();
            case EXTENDED_IDLE, RETRACTED -> HopperParamsNT.idleRPS.getValue();
        };
    }
}
