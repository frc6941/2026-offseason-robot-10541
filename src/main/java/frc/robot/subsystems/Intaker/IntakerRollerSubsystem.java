package frc.robot.subsystems.Intaker;


import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.MotorSubsystem;
import lib.ironpulse.subsystem.SubsystemConfig;

import frc.robot.subsystems.Configs.*;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;

public class IntakerRollerSubsystem extends SubsystemBase{


    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> roller;
    public IntakerRollerSubsystem(
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> roller) {
        this.roller = roller;
    }


}
