// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.AutoAimCommand;
import java.util.function.Supplier;
import frc.robot.commands.Autos;
import frc.robot.commands.auto.AutoPoints;
import frc.robot.commands.auto.AutoBuilder;
import frc.robot.commands.auto.AutoCommands;
import frc.robot.commands.auto.AutoSelector;
import frc.robot.subsystems.Configs.SwerveMK5Config;
import frc.robot.subsystems.Shooter.HoodConfig;
import frc.robot.subsystems.Hopper.HopperConfig;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import frc.robot.subsystems.Intaker.*;
import frc.robot.subsystems.Shooter.HoodParamsNT;
import frc.robot.subsystems.Shooter.ShooterConfig;
import frc.robot.subsystems.Shooter.ShooterParamsNT;
import frc.robot.subsystems.Shooter.ShootingSuperstructure;
import lib.ironpulse.indicator.IndicatorIO;
import lib.ironpulse.indicator.IndicatorIOARGB;
import lib.ironpulse.indicator.IndicatorIOSim;
import lib.ironpulse.indicator.IndicatorSubsystem;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorIOSim;
import lib.ironpulse.io.MotorIOTalonFX;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.limelight.DeviationParamSources;
import lib.ironpulse.limelight.LimelightIOConfig;
import lib.ironpulse.limelight.LimelightIOReal;
import lib.ironpulse.limelight.LimelightSubsystem;
import lib.ironpulse.limelight.commands.LimelightAlignToTag;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.mk5n.ImuIOPigeon;
import lib.ironpulse.swerve.mk5n.SwerveModuleIOMK5N;
import lib.ironpulse.swerve.sim.ImuIOSim;
import lib.ironpulse.swerve.sim.SwerveModuleIOSimpleSim;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private boolean isReal = RobotBase.isReal();

  
  private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> intakerRoller = buildIntakerRoller();
  private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> intakerPivot = buildIntakerPivot();
  private final IntakerSubsystem intaker = new IntakerSubsystem(intakerRoller, intakerPivot);
  private final Swerve swerve = buildSwerve();
  private final HopperSubsystem hopperSubsystem = buildHopper();
  private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooterSubsystem = buildShooter();
  private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hoodSubsystem = buildHood();
  private final ShootingSuperstructure shootingSuperstructure =
      new ShootingSuperstructure(shooterSubsystem, hoodSubsystem, hopperSubsystem, swerve);
  private final AutoBuilder autoBuilder = new AutoBuilder(intaker, swerve, shootingSuperstructure);
  private final LimelightSubsystem limelightSubsystem = buildLimelight();
  private final IndicatorSubsystem indicator = buildIndicator();
  private final RobotMechanism3d mechanism3d = new RobotMechanism3d(hoodSubsystem, intaker);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(0);

  private final AutoSelector autoSelector;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    intaker.setDefaultCommand();
    hopperSubsystem.configureDefaultCommand();
    AutoBuilder.configure(swerve);
    configureBindings();
    autoSelector = new AutoSelector(autoBuilder, Autos.driveForward(swerve));
    new Trigger(() -> true).onTrue(Commands.run(this::updateIndicator));
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
    // Competition-style hood homing: zero on teleop enable so position control starts from a known reference.
    new Trigger(DriverStation::isTeleopEnabled).onTrue(shootingSuperstructure.zeroHood());

    // Driver-triggered intake pivot homing. Keep manual until the team confirms the mechanism can always
    // safely drive into its hard stop on enable.
    driverController.povLeft().onTrue(intaker.zeroCommand());

    // Intake: right trigger deploys + intakes while held, retracts on release.
    // (Hopper feeds automatically off the intake state machine via its default command.)
    driverController.rightTrigger().onTrue(intaker.runIntake());
    driverController.rightTrigger().onFalse(intaker.runRetract());
    // Outtake/reverse: left trigger deploys + outtakes while held, retracts on release
    driverController.leftTrigger().onTrue(intaker.runExtendedReverse());
    driverController.leftTrigger().onFalse(intaker.runRetract());

    // Swerve
    swerve.setDefaultCommand(SwerveCommands.driveWithJoystick(swerve, 
    () -> -driverController.getLeftY(), 
    () -> -driverController.getLeftX(), 
    () -> -driverController.getRightX(), 
    swerve::getEstimatedPose, 
    MetersPerSecond.of(0.03), 
    DegreesPerSecond.of(12)));

    // Shooter and hood (fixed angle) — feed only once shooter is up to speed
    driverController.rightBumper().whileTrue(Commands.parallel(
        shooterSubsystem.runVelVolt(() -> edu.wpi.first.units.Units.RotationsPerSecond.of(ShooterParamsNT.shootRPS.getValue())),
        hoodSubsystem.runMotionMagic(HoodConfig.HOOD_MAX_ANGLE),
        Commands.waitUntil(shooterSubsystem::velocityAtGoal)
            .andThen(hopperSubsystem.feed())
    ));
    driverController.rightBumper().onFalse(Commands.sequence(
        Commands.parallel(
            shooterSubsystem.runVelVolt(() -> edu.wpi.first.units.Units.RotationsPerSecond.of(ShooterParamsNT.idleRPS.getValue())),
            hoodSubsystem.runMotionMagic(HoodConfig.HOOD_STOW_ANGLE)),
        indicator.indicateWithTimeout(IndicatorIO.Patterns.AFTER_SHOOTING, 0.5)));

    // Reset odometry to vision → one-shot correction when the driver notices drift.
    // Also fires once at the start of autonomous.
    new Trigger(DriverStation::isAutonomousEnabled)
        .onTrue(Commands.runOnce(() -> limelightSubsystem.resetPoseFromVision("limelight")));
    driverController.x()
        .onTrue(Commands.runOnce(() -> limelightSubsystem.resetPoseFromVision("limelight")));

    // Precision align to whatever AprilTag the Limelight sees — hold Y, release to stop.
    // Desired offset: 0.5 m in front of the tag, facing it (kPi = look at the tag).
    driverController.y().whileTrue(new LimelightAlignToTag(
        swerve, limelightSubsystem, "limelight", -1,
        new Transform2d(0.5, 0.0, Rotation2d.kPi)));

    // Test-only auto/pathfinding trigger. In keyboard sim this is typically mapped to X.
    driverController.b().onTrue(
        AutoCommands.pathfindToBluePose(
            AutoPoints.OUTPOST,
            AutoCommands.TRANSIT_CONSTRAINTS,
            0.0));

    // Auto-aim: drivetrain rotates to face the hub (yaw), while the shooting superstructure tracks
    // distance to set hood angle + flywheel speed and feeds once chassis/hood/flywheel are all ready.
    driverController.a().whileTrue(Commands.parallel(
        new AutoAimCommand(swerve, () -> -driverController.getLeftY(), () -> -driverController.getLeftX(),
                          shootingSuperstructure::aimHeading,
                          shootingSuperstructure::aimHeadingRateRadPerSec),
        shootingSuperstructure.aimAndShoot()
    ));
    driverController.a().onFalse(Commands.sequence(
        shootingSuperstructure.idle(),
        indicator.indicateWithTimeout(IndicatorIO.Patterns.AFTER_SHOOTING, 0.5)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelector.getCommand();
  }

  public void updateDashboard() {
    autoSelector.updateDashboard();
  }

  public String getAutoSelectionSummary() {
    return autoSelector.getSelectionSummary();
  }

  /**
   * Evaluates subsystem states each cycle and sets the indicator pattern accordingly.
   *
   * <p>Priority order (higher wins when multiple states overlap):
   * <ol>
   *   <li>{@link IndicatorIO.Patterns#AUTO Auto} — robot is in autonomous mode</li>
   *   <li>{@link IndicatorIO.Patterns#SHOOTING Shooting} — flywheel spinning up, not at speed yet</li>
   *   <li>{@link IndicatorIO.Patterns#HOLD_SHOOTING HoldShooting} — flywheel + hood ready, waiting for feed</li>
   *   <li>{@link IndicatorIO.Patterns#INTAKE Intake} — intaker is intaking, feeding, or reversing</li>
   *   <li>{@link IndicatorIO.Patterns#RED_ALLIANCE Red} / {@link IndicatorIO.Patterns#BLUE_ALLIANCE Blue} — disabled, show alliance</li>
   *   <li>{@link IndicatorIO.Patterns#NORMAL Normal} — fallback (teleop driving)</li>
   * </ol>
   */
  public void updateIndicator() {
    // Don't clobber a command-driven transient pattern (e.g. AFTER_SHOOTING flash).
    // Commands set outsideDefault = true while they own the pattern.
    if (indicator.isOutsideDefault()) {
      return;
    }

    // --- 1. Autonomous ---
    if (DriverStation.isAutonomousEnabled()) {
      indicator.setPattern(IndicatorIO.Patterns.AUTO);
      return;
    }

    // --- 2 & 3. Shooting pipeline ---
    // Detect whether the shooter is actively being commanded above idle.
    double shooterSetpointRPS =
        shooterSubsystem.getCurrSetpoint().in(edu.wpi.first.units.Units.RotationsPerSecond);
    boolean shooterActive = shooterSetpointRPS > ShooterParamsNT.idleRPS.getValue() + 1.0;

    if (shooterActive) {
      if (shooterSubsystem.velocityAtGoal() && hoodSubsystem.positionAtGoal()) {
        // Flywheel at speed + hood on target → ready to feed
        indicator.setPattern(IndicatorIO.Patterns.HOLD_SHOOTING);
      } else {
        // Still spinning up
        indicator.setPattern(IndicatorIO.Patterns.SHOOTING);
      }
      return;
    }

    // --- 4. Intaker-deployed states ---
    var intakeMode = intaker.getCurrentMode();
    if (intakeMode == IntakerConfig.IntakeMode.INTAKING
        || intakeMode == IntakerConfig.IntakeMode.FEEDING
        || intakeMode == IntakerConfig.IntakeMode.EXTENDED_REVERSE
        || intakeMode == IntakerConfig.IntakeMode.RETRACTED_FEEDING) {
      indicator.setPattern(IndicatorIO.Patterns.INTAKE);
      return;
    }

    // --- 5. Disabled → alliance colour ---
    if (DriverStation.isDisabled()) {
      var alliance = DriverStation.getAlliance();
      indicator.setPattern(
          alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red
              ? IndicatorIO.Patterns.RED_ALLIANCE
              : IndicatorIO.Patterns.BLUE_ALLIANCE);
      return;
    }

    // --- 6. Fallback ---
    indicator.setPattern(IndicatorIO.Patterns.NORMAL);
  }

 



  private VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> buildIntakerRoller() {
      return new VelocityMotorSubsystem<>(
              IntakerConfig.INTAKER_ROLLER_CONFIG,
              new MotorInputsAutoLogged(),
              isReal
                      ? new MotorIOTalonFX(IntakerConfig.INTAKER_ROLLER_CONFIG)
                      : new MotorIOSim(IntakerConfig.INTAKER_ROLLER_CONFIG),
              IntakerRollerParamsNT.asVelocityParamSources());
  }

    private PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> buildIntakerPivot() {
        return new PositionMotorSubsystem<>(
                IntakerConfig.INTAKER_PIVOT_CONFIG,
                new MotorInputsAutoLogged(),
                isReal
                        ? new MotorIOTalonFX(IntakerConfig.INTAKER_PIVOT_CONFIG)
                        : new MotorIOSim(IntakerConfig.INTAKER_PIVOT_CONFIG),
                IntakerPivotParamsNT.asPositionParamSources(),
                Degrees.of(0),
                IntakerConfig.INTAKER_ANGLE_PER_ROTATION);
    }

  private HopperSubsystem buildHopper() {
    return new HopperSubsystem(
        intaker,
        HopperConfig.HOPPER_CONFIG,
        new MotorInputsAutoLogged(),
        RobotBase.isReal()
            ? new MotorIOTalonFX(HopperConfig.HOPPER_CONFIG)
            : new MotorIOSim(HopperConfig.HOPPER_CONFIG));
  }

  private VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> buildShooter() {
    return new VelocityMotorSubsystem<>(
        ShooterConfig.SHOOTER_CONFIG,
        new MotorInputsAutoLogged(),
        RobotBase.isReal()
            ? new MotorIOTalonFX(ShooterConfig.SHOOTER_CONFIG)
            : new MotorIOSim(ShooterConfig.SHOOTER_CONFIG),
        ShooterParamsNT.asVelocityParamSources());
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

  private IndicatorSubsystem buildIndicator() {
    return new IndicatorSubsystem(
        RobotBase.isReal()
            ? new IndicatorIOARGB(/* PWM port */ 9, /* LED count */ 60)
            : new IndicatorIOSim());
  }

  private PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> buildHood() {
    return new PositionMotorSubsystem<>(
        HoodConfig.HOOD_CONFIG,
        new MotorInputsAutoLogged(),
        RobotBase.isReal()
            ? new MotorIOTalonFX(HoodConfig.HOOD_CONFIG)
            : new MotorIOSim(HoodConfig.HOOD_CONFIG),
        HoodParamsNT.asPositionParamSources(),
        HoodConfig.HOOD_MIN_ANGLE,
        HoodConfig.HOOD_ANGLE_PER_ROTATION);
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
