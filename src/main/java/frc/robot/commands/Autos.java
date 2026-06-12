// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.ironpulse.swerve.Swerve;

public final class Autos {
  

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Command driveForward(Swerve swerve) {
    return Commands.run(() -> swerve.runTwist(new ChassisSpeeds(1.0, 0.0, 0.0)), swerve)
                        .withTimeout(2.0)
                        .andThen(Commands.runOnce(swerve::runStop, swerve));
  }
}
