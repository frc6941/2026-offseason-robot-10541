package frc.robot.subsystems.FloorRoller;

import edu.wpi.first.wpilibj2.command.Command;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.MotorSubsystem;
import lib.ironpulse.subsystem.SubsystemConfig;

public class FloorRollerSubsystem extends MotorSubsystem<MotorInputsAutoLogged, MotorIO>{

    private static final double IDLE_SPEED = 0.2;
    private static final double FEED_SPEED = 0.6;

    public FloorRollerSubsystem(SubsystemConfig config, MotorInputsAutoLogged inputs, MotorIO io){
        super(config, inputs, io);
    }

    public Command idle() {
        return runDutyCycle(IDLE_SPEED);
    }

    public Command feed(){
        return runDutyCycle(FEED_SPEED);
    }

    public Command reverse(){
        return runDutyCycle(-FEED_SPEED);
    }
}
