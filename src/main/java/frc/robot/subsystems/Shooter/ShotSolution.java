package frc.robot.subsystems.Shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** Distance-table output used by the hood and upper/lower shooter commands. */
public record ShotSolution(Angle hoodAngle, AngularVelocity shooterSpeed) {}
