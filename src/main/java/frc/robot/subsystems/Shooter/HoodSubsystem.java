package frc.robot.subsystems.Shooter;


import static edu.wpi.first.units.Units.Degrees;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.position.PositionParamSources;

public class HoodSubsystem extends PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle>{
    private final static Angle MIN_ANGLE = Degrees.of(0);
    private final static Angle MAX_ANGLE = Degrees.of(30);
    private final static Angle DEGREES_PER_ROTATION = Degrees.of(360);

    public HoodSubsystem(SubsystemConfig config, MotorInputsAutoLogged inputs, MotorIO io, PositionParamSources params){
        super(config, inputs, io, params, MIN_ANGLE, DEGREES_PER_ROTATION);
    }


    public Command setFlat(){
        return runMotionMagic(MIN_ANGLE);
    }

    public Command setMaxAngle(){
        return runMotionMagic(MAX_ANGLE);
    }

    public Command setAngle(Angle angle){
        return runMotionMagic(angle);
    }
}
