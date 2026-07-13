// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoTrenchCommand;
import frc.robot.commands.DefaultAuto;
import frc.robot.commands.auto.AutoBuilder;
import frc.robot.commands.auto.AutoSelector;
import frc.robot.subsystems.Configs.SwerveMK5Config;
import frc.robot.subsystems.Hopper.HopperConfig;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import frc.robot.subsystems.Intaker.*;
import frc.robot.subsystems.Shooter.ShooterConfig;
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
import lib.ironpulse.math.rbd.TransformRecorder;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.mk5n.ImuIOPigeon;
import lib.ironpulse.swerve.mk5n.SwerveModuleIOMK5N;
import lib.ironpulse.swerve.sim.ImuIOSim;
import lib.ironpulse.swerve.sim.SwerveModuleIOSimpleSim;
import lib.ironpulse.utils.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private boolean isReal = RobotBase.isReal();

    // Limelight device id — must exactly match the limelight's hostname (web UI -> Settings ->
    // Hostname). Single source of truth so the IO registration and every lookup can't drift apart.
    private static final String LIMELIGHT_NAME = "limelight-a";

    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> intakerRoller =
            buildIntakerRoller();
    private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> intakerPivot =
            buildIntakerPivot();
    private final IntakerSubsystem intaker = new IntakerSubsystem(intakerRoller, intakerPivot);
    private final Swerve swerve = buildSwerve();
    private final HopperSubsystem hopperSubsystem = buildHopper();
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooterUpperSubsystem =
            buildShooterDrum();
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooterLowerSubsystem =
            buildShooterFeed();
    private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hoodSubsystem =
            buildHood();
    private final ShootingSuperstructure shootingSuperstructure =
            new ShootingSuperstructure(
                    shooterUpperSubsystem,
                    shooterLowerSubsystem,
                    hoodSubsystem,
                    hopperSubsystem,
                    swerve);
    private final AutoBuilder autoBuilder =
            new AutoBuilder(intaker, swerve, shootingSuperstructure);
    private final LimelightSubsystem limelightSubsystem = buildLimelight();
    private final IndicatorSubsystem indicator = buildIndicator();

    @SuppressWarnings("unused")
    private final RobotMechanism3d mechanism3d = new RobotMechanism3d(hoodSubsystem, intaker);

    @SuppressWarnings("unused")
    private final FieldCoreBridge fieldCoreBridge =
            RobotBase.isSimulation()
                    ? new FieldCoreBridge(
                            swerve,
                            intaker,
                            hopperSubsystem,
                            shooterUpperSubsystem,
                            shooterLowerSubsystem,
                            hoodSubsystem)
                    : null;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController OperatorController = new CommandXboxController(1);

    private final AutoSelector autoSelector;
    private int hubTargetModeRequests = 0;
    private AutoAimCommand.TargetMode targetModeBeforeHubRequests = AutoAimCommand.TargetMode.AUTO;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Start the swerve odometry sampler thread now that all modules + the Pigeon have
        // registered
        // their signals (during field init above). The lib creates the thread lazily but never
        // starts
        // it; without this the odometry queues stay empty and the pose estimator never updates.
        // No-op
        // in sim (no MK5N modules -> syncThread is null).
        SwerveModuleIOMK5N.startSyncThread();
        intaker.setDefaultCommand();
        hopperSubsystem.configureDefaultCommand();
        shootingSuperstructure.configureDefaultCommands();
        // Intake zeroing stays manual-only on D-pad Left.
        AutoBuilder.configure(swerve);
        configureBindings();
        autoSelector = new AutoSelector(autoBuilder, DefaultAuto.driveForward(swerve));
        new Trigger(() -> true).onTrue(Commands.run(this::updateIndicator));

        // Publish the Field2d ("Field") for Elastic + hook PathPlanner active-path logging
        // (one-time).
        FieldPublisher.init();
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
        // Manual intake pivot hard-stop zero.
        OperatorController.povLeft().onTrue(intaker.zeroCommand());
        OperatorController.povDown().whileTrue(intaker.runExtendedReverse());

        // Intake: left trigger runs intake while held.
        // (Hopper feeds automatically off the intake state machine via its default command.)
        driverController.leftTrigger().whileTrue(intaker.runIntakeContinuous());

        // Swerve
        // Pass the DRIVER-relative robot pose (not the raw world pose) so "forward" on the stick
        // means
        // away-from-driver on both alliances. The 180deg flip lives in the DriverStation frame
        // inside
        // RobotStateRecorder, not in a hand-written shouldFlip() here. (comp-bot does the same.)
        swerve.setDefaultCommand(
                SwerveCommands.driveWithJoystick(
                        swerve,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX(),
                        RobotStateRecorder::getPoseDriverRobotCurrent,
                        MetersPerSecond.of(0.03),
                        DegreesPerSecond.of(12)));

        // Field-relative heading zero: place the robot on the field facing the correct direction
        // and
        // press Start. Seeds the Pigeon to field yaw (0, or 180 on the flipped alliance) so
        // MegaTag2 —
        // which only solves X/Y from the gyro heading — corrects position correctly from then on.
        driverController
                .start()
                .onTrue(
                        Commands.sequence(
                                        SwerveCommands.resetAngle(
                                                        swerve,
                                                        () ->
                                                                AllianceFlipUtil.shouldFlip()
                                                                        ? Rotation2d.k180deg
                                                                        : Rotation2d.kZero)
                                                .alongWith(
                                                        Commands.runOnce(
                                                                () ->
                                                                        RobotStateRecorder
                                                                                .getInstance()
                                                                                .resetTransform(
                                                                                        TransformRecorder
                                                                                                .kFrameWorld,
                                                                                        TransformRecorder
                                                                                                .kFrameRobot))),
                                        Commands.runOnce(
                                                limelightSubsystem::requestInternalIMUReseedAll))
                                .alongWith(
                                        indicator.indicateWithTimeout(
                                                IndicatorIO.Patterns.RESET_ODOM, 1)));

        // Y aims the drivetrain at the hub. X/B temporarily run AutoTrench. RT shoots only, so
        // drivers can hold Y + RT together for the old aim-and-shoot behavior.
        driverController.y().whileTrue(aimAtHubCommand());

        driverController.x().whileTrue(autoTrenchCommand());
        driverController.b().whileTrue(autoTrenchCommand());

        driverController.a().onTrue(intaker.runRetract());

        driverController.rightTrigger().whileTrue(shootAtHubCommand());
        driverController
                .rightTrigger()
                .onFalse(
                        Commands.parallel(
                                intaker.returnPivotToIdleFast(),
                                Commands.sequence(
                                        shootingSuperstructure.idle().withTimeout(0.02),
                                        indicator.indicateWithTimeout(
                                                IndicatorIO.Patterns.AFTER_SHOOTING, 0.5))));
    }

    private Command aimAtHubCommand() {
        return Commands.parallel(
                holdHubTargetMode(),
                new AutoAimCommand(
                        swerve,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        shootingSuperstructure::aimHeading,
                        shootingSuperstructure::aimHeadingRateRadPerSec));
    }

    private Command shootAtHubCommand() {
        return Commands.parallel(
                holdHubTargetMode(),
                shootingSuperstructure.aimAndShoot(),
                intaker.holdRetractedFeedPosition());
    }

    private Command autoTrenchCommand() {
        return new AutoTrenchCommand(swerve, () -> -driverController.getLeftY());
    }

    private Command holdHubTargetMode() {
        return Commands.startEnd(
                () -> {
                    if (hubTargetModeRequests == 0) {
                        targetModeBeforeHubRequests = AutoAimCommand.getTargetMode();
                        AutoAimCommand.setTargetMode(AutoAimCommand.TargetMode.HUB);
                    }
                    hubTargetModeRequests++;
                },
                () -> {
                    hubTargetModeRequests = Math.max(0, hubTargetModeRequests - 1);
                    if (hubTargetModeRequests == 0) {
                        AutoAimCommand.setTargetMode(targetModeBeforeHubRequests);
                    }
                });
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
        // Feed the swerve pose into the transform tree (World->Robot). This is the single place the
        // robot's world pose enters RobotStateRecorder; everything alliance-aware (driver-relative
        // driving, flipped targets) derives from here. Runs every loop incl. disabled.
        RobotStateRecorder.putRobotState(
                Seconds.of(Timer.getTimestamp()),
                swerve.getEstimatedPose(),
                swerve.getChassisSpeeds(),
                swerve.getChassisSpeedsCmd(),
                RadiansPerSecond.of(swerve.getYawVelocityRadPerSec()));
        RobotStateRecorder.periodic();

        autoSelector.updateDashboard();

        // Robot pose on the Field2d for Elastic (the path/target come from PathPlanner's
        // callbacks).
        FieldPublisher.setRobotPose(swerve.getEstimatedPose().toPose2d());

        // Vision ghost — re-homed out of the vendored lib so future lib copies stay drop-in. Logs
        // the
        // tag-derived robot pose as a Pose2d[], hidden (empty array) when there's no target so it
        // doesn't snap to the field origin. Bind a translucent robot to Vision/Ghost in
        // AdvantageScope.
        Pose2d visionPose = limelightSubsystem.getPose(LIMELIGHT_NAME);
        Logger.recordOutput(
                "Vision/Ghost",
                visionPose.equals(new Pose2d()) ? new Pose2d[0] : new Pose2d[] {visionPose});
    }

    public String getAutoSelectionSummary() {
        return autoSelector.getSelectionSummary();
    }

    /**
     * Evaluates subsystem states each cycle and sets the indicator pattern accordingly.
     *
     * <p>Priority order (higher wins when multiple states overlap):
     *
     * <ol>
     *   <li>{@link IndicatorIO.Patterns#AUTO Auto} — robot is in autonomous mode
     *   <li>{@link IndicatorIO.Patterns#SHOOTING Shooting} — flywheel spinning up, not at speed yet
     *   <li>{@link IndicatorIO.Patterns#HOLD_SHOOTING HoldShooting} — flywheel + hood ready,
     *       waiting for feed
     *   <li>{@link IndicatorIO.Patterns#INTAKE Intake} — intaker is intaking, feeding, or reversing
     *   <li>{@link IndicatorIO.Patterns#RED_ALLIANCE Red} / {@link
     *       IndicatorIO.Patterns#BLUE_ALLIANCE Blue} — disabled, show alliance
     *   <li>{@link IndicatorIO.Patterns#NORMAL Normal} — fallback (teleop driving)
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
        boolean shooterActive = shootingSuperstructure.isShooterActive();

        if (shooterActive) {
            if (shootingSuperstructure.shooterAtGoal() && shootingSuperstructure.hoodAtGoal()) {
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
                IntakerConfig.IntakerRollerParams.asVelocityParamSources());
    }

    private PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> buildIntakerPivot() {
        return new PositionMotorSubsystem<>(
                IntakerConfig.INTAKER_PIVOT_CONFIG,
                new MotorInputsAutoLogged(),
                isReal
                        ? new MotorIOTalonFX(IntakerConfig.INTAKER_PIVOT_CONFIG)
                        : new MotorIOSim(IntakerConfig.INTAKER_PIVOT_CONFIG),
                IntakerConfig.IntakerPivotParams.asPositionParamSources(),
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

    private VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> buildShooterDrum() {
        return new VelocityMotorSubsystem<>(
                ShooterConfig.SHOOTER_DRUM_CONFIG,
                new MotorInputsAutoLogged(),
                RobotBase.isReal()
                        ? new MotorIOTalonFX(ShooterConfig.SHOOTER_DRUM_CONFIG)
                        : new MotorIOSim(ShooterConfig.SHOOTER_DRUM_CONFIG),
                ShooterConfig.ShooterUpperParams.asVelocityParamSources());
    }

    private VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> buildShooterFeed() {
        return new VelocityMotorSubsystem<>(
                ShooterConfig.SHOOTER_FEED_CONFIG,
                new MotorInputsAutoLogged(),
                RobotBase.isReal()
                        ? new MotorIOTalonFX(ShooterConfig.SHOOTER_FEED_CONFIG)
                        : new MotorIOSim(ShooterConfig.SHOOTER_FEED_CONFIG),
                ShooterConfig.ShooterLowerParams.asVelocityParamSources());
    }

    private LimelightSubsystem buildLimelight() {
        LimelightIOConfig config =
                LimelightIOConfig.builder()
                        .name(LIMELIGHT_NAME)
                        .useMegaTag2(true)
                        .mountPosition(LimelightIOConfig.MountPosition.ON_ROBOT)
                        .build();

        LimelightIOReal io =
                new LimelightIOReal(
                        config,
                        swerve::getIMUYaw,
                        swerve::getYawVelocityRadPerSec,
                        () -> false,
                        // TODO: tune vision std-devs on the real robot.
                        new DeviationParamSources() {
                            public double xStdDev() {
                                return 0.7;
                            }

                            public double yStdDev() {
                                return 0.7;
                            }

                            public double zStdDev() {
                                return 9999.0;
                            }

                            public double angleStdDev() {
                                return 999999999.0;
                            }

                            public double imuCorrectionReliabilityThreshold() {
                                return 0.9;
                            }
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
                ShooterConfig.HOOD_CONFIG,
                new MotorInputsAutoLogged(),
                RobotBase.isReal()
                        ? new MotorIOTalonFX(ShooterConfig.HOOD_CONFIG)
                        : new MotorIOSim(ShooterConfig.HOOD_CONFIG),
                ShooterConfig.HoodParams.asPositionParamSources(),
                ShooterConfig.HOOD_STOW_ANGLE,
                ShooterConfig.HOOD_ANGLE_PER_ROTATION);
    }

    private Swerve buildSwerve() {
        if (RobotBase.isReal()) {
            // Build the modules BEFORE the IMU. SwerveModuleIOMK5N lazily creates the shared
            // PhoenixSynchronizationThread; ImuIOPigeon then grabs it via getSyncThread() to
            // register its
            // yaw + the odometry timestamp queue. If the IMU is constructed first, getSyncThread()
            // returns
            // null, the IMU silently skips registration, and the pose estimator never gets sampled
            // timestamps — frozen pose even though the wheels still drive. (Arg order to Swerve
            // unchanged.)
            var module0 = new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 0);
            var module1 = new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 1);
            var module2 = new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 2);
            var module3 = new SwerveModuleIOMK5N(SwerveMK5Config.kRealConfig, 3);
            var imu = new ImuIOPigeon(SwerveMK5Config.kRealConfig, SwerveMK5Config.pigeonConfig);
            return new Swerve(SwerveMK5Config.kRealConfig, imu, module0, module1, module2, module3);
        } else {
            return new Swerve(
                    SwerveMK5Config.kSimConfig,
                    new ImuIOSim(),
                    new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 0),
                    new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 1),
                    new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 2),
                    new SwerveModuleIOSimpleSim(SwerveMK5Config.kSimConfig, 3));
        }
    }
}
