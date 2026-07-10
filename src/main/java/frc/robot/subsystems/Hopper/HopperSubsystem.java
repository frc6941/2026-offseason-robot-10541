package frc.robot.subsystems.Hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
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

    // Pass suppliers, not snapshot values: a Command built with a captured
    // RotationsPerSecond.of(...)
    // would freeze the RPS to its construction-time value and ignore live NT edits. A supplier
    // re-reads every loop.
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
        // Re-evaluate the intake mode every loop. A single runVelTC re-reads getCurrentMode() (and
        // the NT RPS) each iteration. Commands.select would latch the mode at initialize() and,
        // because the selected runVelTC never finishes, never re-run the selector — freezing the
        // hopper at its boot-time mode until some other command interrupted the default.
        setDefaultCommand(runVelTC(this::defaultVelocity));
    }

    private AngularVelocity defaultVelocity() {
        return switch (intaker.getCurrentMode()) {
            case INTAKING, FEEDING -> RotationsPerSecond.of(HopperParamsNT.feedRPS.getValue());
            case RETRACTED_FEEDING, EXTENDED_REVERSE ->
                    RotationsPerSecond.of(-HopperParamsNT.feedRPS.getValue());
            case EXTENDED_IDLE, RETRACTED ->
                    RotationsPerSecond.of(HopperParamsNT.idleRPS.getValue());
        };
    }
}
