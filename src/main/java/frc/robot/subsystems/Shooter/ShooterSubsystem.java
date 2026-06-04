package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityParamSources;

public class ShooterSubsystem extends VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> {

    private final static AngularVelocity SHOOT_VELOCITY = RotationsPerSecond.of(80);
    private final static AngularVelocity IDLE_VELOCITY = RotationsPerSecond.of(0);

    public ShooterSubsystem(SubsystemConfig config, MotorInputsAutoLogged inputs, MotorIO io, VelocityParamSources params){
        super(config, inputs, io, params);
    }

    public Command spinUp(){
        return runVelVolt(SHOOT_VELOCITY);
    }

    public Command stop(){
        return runVelVolt(IDLE_VELOCITY);
    }
}
