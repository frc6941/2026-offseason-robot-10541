package frc.robot.subsystems.Intaker;


import edu.wpi.first.wpilibj2.command.Command;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.MotorSubsystem;
import lib.ironpulse.subsystem.SubsystemConfig;

public class IntakerRollerSubsystem extends MotorSubsystem<MotorInputsAutoLogged, MotorIO> {

    public IntakerRollerSubsystem(SubsystemConfig config, MotorInputsAutoLogged inputs, MotorIO io) {
        super(config, inputs, io);
    }
   
    public Command runintake() {

    }

    public Command outtake() {
        return runDutyCycle(-0.7);
    }

}
