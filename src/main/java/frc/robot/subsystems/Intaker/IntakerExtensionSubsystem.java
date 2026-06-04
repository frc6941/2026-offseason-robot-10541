package frc.robot.subsystems.Intaker;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.position.PositionParamSources;

public class IntakerExtensionSubsystem extends PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Distance>{

    private static final Distance RETRACTED = Meters.of(0.0);
    private static final Distance EXTENDED = Meters.of(0.3);
    private static final Distance METERS_PER_ROTATION = Meters.of(0.6);

    public IntakerExtensionSubsystem(SubsystemConfig config, MotorInputsAutoLogged inputs, MotorIO io, PositionParamSources params){
        super(config, inputs, io, params, RETRACTED, METERS_PER_ROTATION);
    }

    public Command extend() {
        return runMotionMagic(EXTENDED);
    }

    public Command retract(){
        return runMotionMagic(RETRACTED);
    }

}
