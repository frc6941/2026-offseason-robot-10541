package frc.robot.subsystems.Shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * A complete stationary shot solution for the drum shooter.
 *
 * <p>The drum shooter is fixed to the chassis, so a full aim is three DOF: chassis heading (handled
 * by {@link frc.robot.commands.AutoAimCommand}), hood pitch, and flywheel speed. This record carries
 * the two mechanism components; heading is realized by the drivetrain separately.
 */
public record ShotSolution(Angle hoodAngle, AngularVelocity shooterSpeed) {}
