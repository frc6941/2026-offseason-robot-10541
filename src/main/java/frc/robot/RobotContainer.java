// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAimCommand;
import frc.robot.subsystems.Configs.SwerveMK5Config;
import frc.robot.subsystems.FloorRoller.FloorRollerSubsystem;
import frc.robot.subsystems.Intaker.IntakerExtensionSubsystem;
import frc.robot.subsystems.Intaker.IntakerRollerSubsystem;
import frc.robot.subsystems.Shooter.HoodSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import lib.ironpulse.io.MotorIOSim;
import lib.ironpulse.io.MotorIOTalonFX;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.limelight.DeviationParamSources;
import lib.ironpulse.limelight.LimelightIOConfig;
import lib.ironpulse.limelight.LimelightIOReal;
import lib.ironpulse.limelight.LimelightSubsystem;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.subsystem.position.PositionParamSources;
import lib.ironpulse.subsystem.velocity.VelocityParamSources;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.mk5n.ImuIOPigeon;
import lib.ironpulse.swerve.mk5n.SwerveModuleIOMK5N;
import lib.ironpulse.swerve.sim.ImuIOSim;
import lib.ironpulse.swerve.sim.SwerveModuleIOSimpleSim;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final IntakerRollerSubsystem intakerRollerSubsystem = buildIntakerRoller();
  private final IntakerExtensionSubsystem intakerExtensionSubsystem = buildIntakerExtension();
  private final Swerve swerve = buildSwerve();
  private final FloorRollerSubsystem floorRollerSubsystem = buildFloorRoller();
  private final ShooterSubsystem shooterSubsystem = buildShooter();
  private final HoodSubsystem hoodSubsystem = buildHood();
  private final LimelightSubsystem limelightSubsystem = buildLimelight();
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Intake and outake
    driverController.rightTrigger().whileTrue(Commands.parallel(intakerExtensionSubsystem.extend(),intakerRollerSubsystem.intake(), floorRollerSubsystem.feed()));
    driverController.rightTrigger().onFalse(intakerExtensionSubsystem.retract());
    driverController.leftTrigger().whileTrue(intakerRollerSubsystem.outtake());

    // Swerve
    swerve.setDefaultCommand(SwerveCommands.driveWithJoystick(swerve, driverController::getLeftX, driverController::getLeftY, driverController::getRightX, swerve::getEstimatedPose, MetersPerSecond.of(0.1), DegreesPerSecond.of(5)));

    // Roller Floor
    floorRollerSubsystem.setDefaultCommand(floorRollerSubsystem.idle());

    // Shooter and hood (fixed angle)
    driverController.rightBumper().whileTrue(Commands.parallel(shooterSubsystem.spinUp(), hoodSubsystem.setMaxAngle(), floorRollerSubsystem.feed()));
    driverController.rightBumper().onFalse(Commands.parallel(shooterSubsystem.stop(), hoodSubsystem.setFlat()));

    // Auto-aim: left bumper — swerve rotates to face hub, hood adjusts angle by distance
    driverController.a().whileTrue(Commands.parallel(
        new AutoAimCommand(swerve, driverController::getLeftX, driverController::getLeftY),
        shooterSubsystem.spinUp(),
        hoodSubsystem.runMotionMagic(() -> HoodSubsystem.getAngleForDistance(
            AutoAimCommand.getDistanceToTarget(
                swerve.getEstimatedPose().toPose2d().getTranslation()))),
        floorRollerSubsystem.feed()
    ));
    driverController.a().onFalse(Commands.parallel(shooterSubsystem.stop(), hoodSubsystem.setFlat()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  private IntakerRollerSubsystem buildIntakerRoller() {
    SubsystemConfig config = SubsystemConfig.simpleMotorCfg(
        "intaker_roller", 15, RobotConstants.CANIVORE_CAN_BUS, InvertedValue.CounterClockwise_Positive);
    return new IntakerRollerSubsystem(
        config,
        new MotorInputsAutoLogged(),
        RobotBase.isReal() ? new MotorIOTalonFX(config) : new MotorIOSim(config));
  }

  private IntakerExtensionSubsystem buildIntakerExtension() {
    SubsystemConfig config = SubsystemConfig.simpleMotorCfg(
        "intaker_extension", 14, RobotConstants.CANIVORE_CAN_BUS, InvertedValue.CounterClockwise_Positive);
    return new IntakerExtensionSubsystem(
        config,
        new MotorInputsAutoLogged(),
        RobotBase.isReal() ? new MotorIOTalonFX(config) : new MotorIOSim(config),
        new PositionParamSources() {
          public double kP() { return 0.0; }
          public double kI() { return 0.0; }
          public double kD() { return 0.0; }
        });
  }

  private FloorRollerSubsystem buildFloorRoller() {
    SubsystemConfig config = SubsystemConfig.simpleMotorCfg(
        "floor_roller", 20, RobotConstants.CANIVORE_CAN_BUS, InvertedValue.Clockwise_Positive);
    return new FloorRollerSubsystem(
        config,
        new MotorInputsAutoLogged(),
        RobotBase.isReal() ? new MotorIOTalonFX(config) : new MotorIOSim(config));
  }

  private ShooterSubsystem buildShooter() {
    SubsystemConfig config = SubsystemConfig.builder()
      .name("shooter")
      .mainId(19)
      .mainBus(RobotConstants.CANIVORE_CAN_BUS)
      .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
      .followers(new SubsystemConfig.FollowerConfig[]{
        SubsystemConfig.FollowerConfig.builder()
            .id(20)
            .bus(RobotConstants.CANIVORE_CAN_BUS)
            .build()
        }
      )
    .build();
    return new ShooterSubsystem(
        config,
        new MotorInputsAutoLogged(),
        RobotBase.isReal() ? new MotorIOTalonFX(config) : new MotorIOSim(config),
        new VelocityParamSources() {
          public double kP() { return 0.0; }
          public double kI() { return 0.0; }
          public double kD() { return 0.0; }
        });
  }

  private LimelightSubsystem buildLimelight() {
    LimelightIOConfig config = LimelightIOConfig.builder()
        .name("limelight")
        .useMegaTag2(true)
        .mountPosition(LimelightIOConfig.MountPosition.ON_ROBOT)
        .build();
    LimelightIOReal io = new LimelightIOReal(
        config,
        swerve::getIMUYaw,
        swerve::getYawVelocityRadPerSec,
        () -> false,
        new DeviationParamSources() {
          public double xStdDev() { return 0.7; }
          public double yStdDev() { return 0.7; }
          public double zStdDev() { return 9999.0; }
          public double angleStdDev() { return 1.0; }
          public double imuCorrectionReliabilityThreshold() { return 0.9; }
        });
    return new LimelightSubsystem(swerve, io);
  }

  private HoodSubsystem buildHood() {
    SubsystemConfig config = SubsystemConfig.simpleMotorCfg(
        "hood", 10, RobotConstants.CANIVORE_CAN_BUS, InvertedValue.CounterClockwise_Positive);
    return new HoodSubsystem(
        config,
        new MotorInputsAutoLogged(),
        RobotBase.isReal() ? new MotorIOTalonFX(config) : new MotorIOSim(config),
        new PositionParamSources() {
          public double kP() { return 0.0; }
          public double kI() { return 0.0; }
          public double kD() { return 0.0; }
        });
  }

  private Swerve buildSwerve() {
    if (RobotBase.isReal()) {
      return new Swerve(
          SwerveMK5Config.kRealConfig,
          new ImuIOPigeon(SwerveMK5Config.kRealConfig, SwerveMK5Config.pigeonConfig),
          new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 0),
          new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 1),
          new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 2),
          new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 3));
    } else {
      return new Swerve(
          SwerveMK5Config.kRealConfig,
          new ImuIOSim(),
          new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 0),
          new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 1),
          new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 2),
          new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 3));
    }
  }
}
