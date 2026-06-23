package frc.robot.subsystems.Intaker;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.position.PositionParamSources;

public class IntakerExtensionSubsystem extends PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle>{

    private static final Angle RETRACTED = Degrees.of(0);
    private static final Angle EXTENDED = Degrees.of(135);


    public IntakerExtensionSubsystem(SubsystemConfig config, MotorInputsAutoLogged inputs, MotorIO io, PositionParamSources params){
        super(config, inputs, io, params, RETRACTED, 1/IntakerConfig.);
    }

    public Command extend() {
        return runMotionMagic(EXTENDED);
    }

    public Command retract(){
        return runMotionMagic(RETRACTED);
    }

}
