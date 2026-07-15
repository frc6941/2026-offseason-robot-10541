// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.RobotConstants.HAS_SWERVE_IO;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoActions;
import frc.robot.auto.AutoFile;
import frc.robot.auto.AutoRoutines;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoTrenchCommand;
import frc.robot.subsystems.Configs.LimeLightConfig;
import frc.robot.subsystems.Configs.SwerveMK5Config;
import frc.robot.subsystems.Hopper.HopperConfig;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import frc.robot.subsystems.Intaker.*;
import frc.robot.subsystems.Shooter.HoodParamsNT;
import frc.robot.subsystems.Shooter.ShooterConfig;
import frc.robot.subsystems.Shooter.ShooterLowerParamsNT;
import frc.robot.subsystems.Shooter.ShooterUpperParamsNT;
import frc.robot.subsystems.Shooter.ShootingSuperstructure;
import lib.ironpulse.indicator.IndicatorIO;
import lib.ironpulse.indicator.IndicatorIOARGB;
import lib.ironpulse.indicator.IndicatorIOSim;
import lib.ironpulse.indicator.IndicatorSubsystem;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorIOSim;
import lib.ironpulse.io.MotorIOTalonFX;
import lib.ironpulse.io.MotorInputsAutoLogged;
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

    private static final double INTAKE_TRIGGER_START_THRESHOLD = 0.1;
    private static final double INTAKE_TRIGGER_MAX_THRESHOLD = 0.6;

    private boolean isReal = RobotBase.isReal();

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
        configureBindings();
        // Autonomous: PathPlanner path-following routines (see frc.robot.auto).
        AutoActions.init(swerve, shootingSuperstructure, intaker);
        AutoRoutines.init(swerve, shootingSuperstructure, intaker);
        AutoFile.init();
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
        // SYSID/test
        /*
        SysIdCommand shooterSysId = new SysIdCommand(intakerRoller);
        driverController.a().whileTrue(shooterSysId.quasistatic(Direction.kForward));
        driverController.b().whileTrue(shooterSysId.quasistatic(Direction.kReverse));
        driverController.x().whileTrue(shooterSysId.dynamic(Direction.kForward));
        driverController.y().whileTrue(shooterSysId.dynamic(Direction.kReverse));
        */

        // Manual intake pivot hard-stop zero.
        OperatorController.povLeft().onTrue(intaker.zeroCommand());
        OperatorController.povDown().whileTrue(intaker.runExtendedReverse());
        OperatorController.povRight().onTrue(hoodSubsystem.zeroCommand());

        driverController.povLeft().onTrue(intaker.zeroCommand());
        driverController.povDown().whileTrue(intaker.runExtendedReverse());
        driverController.povRight().onTrue(hoodSubsystem.zeroCommand());

        driverController.povUp().whileTrue(driveToPoseAutoPilotTestCommand());

        // Intake: left trigger runs intake while held.
        // (Hopper feeds automatically off the intake state machine via its default command.)
        Trigger maxIntakeTrigger =
                new Trigger(
                        () ->
                                driverController.getLeftTriggerAxis()
                                        >= INTAKE_TRIGGER_MAX_THRESHOLD);
        driverController
                .leftTrigger(INTAKE_TRIGGER_START_THRESHOLD)
                .and(maxIntakeTrigger.negate())
                .whileTrue(intaker.runIntakeContinuous());
        maxIntakeTrigger.whileTrue(intaker.runMaxIntakeContinuous());

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

        // Y aims without shooting; RT aims and shoots. Both select the hub inside our alliance
        // zone, or the matching pass corner after crossing out of it.
        driverController.y().whileTrue(aimCommand());

        driverController.x().whileTrue(autoTrenchCommand());
        driverController.b().whileTrue(autoTrenchCommand());

        driverController.a().onTrue(intaker.runRetract());

        // Keep the Autopilot test helper unbound; D-pad Down is reserved for intake reverse.

        // driverController.rightTrigger().whileTrue(shootAtHubCommand());
        // Test: drive to the second-sweep start pose with the AutopilotDriveToPose point-to-point
        // driver (straight beeline). Hold D-pad Down to run, release to abort. Tune
        // Params/Commands/AutopilotDriveToPose live and watch Commands/AutopilotDriveToPose/* in
        // AdvantageScope.
        // driverController.povDown().whileTrue(autopilotDriveToPoseTestCommand());
        driverController.rightTrigger().whileTrue(shootCommand());
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

    private Command aimCommand() {
        return Commands.parallel(
                holdAutomaticTargetMode(),
                new AutoAimCommand(
                        swerve,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        shootingSuperstructure::aimHeading,
                        shootingSuperstructure::aimHeadingRateRadPerSec));
    }

    private Command shootCommand() {
        return Commands.sequence(
                intaker.forceExtendedIdle(),
                Commands.parallel(
                        holdAutomaticTargetMode(),
                        new AutoAimCommand(
                                swerve,
                                () -> -driverController.getLeftY(),
                                () -> -driverController.getLeftX(),
                                shootingSuperstructure::aimHeading,
                                shootingSuperstructure::aimHeadingRateRadPerSec),
                        shootingSuperstructure.aimAndShoot(),
                        shootingSuperstructure
                                .waitForFeedStart()
                                .andThen(intaker.holdRetractedShootPosition())));
    }

    private Command autoTrenchCommand() {
        return new AutoTrenchCommand(swerve, () -> -driverController.getLeftY());
    }

    /**
     * Drives to the (alliance-flipped) second-sweep start pose with the Autopilot point-to-point
     * driver ({@link AutoActions#driveToPoseAutoPilot}). Gains/tolerances come from {@code
     * Params/AutoPilot} and are re-read each press (deferred), so tune on the dashboard and
     * re-press. Deferred also so it builds only after {@link AutoActions#init} has run.
     */
    private Command driveToPoseAutoPilotTestCommand() {
        return Commands.deferredProxy(
                () ->
                        AutoActions.driveToPoseAutoPilot(
                                        AllianceFlipUtil.apply(AutoActions.kSecondSweepStartL))
                                .andThen(
                                        AutoActions.sweepCollectShoot(
                                                "RightTrenchSecondMiddle", true)));
    }

    private Command holdAutomaticTargetMode() {
        return Commands.startEnd(
                () -> AutoAimCommand.setTargetMode(AutoAimCommand.TargetMode.AUTO),
                () -> AutoAimCommand.setTargetMode(AutoAimCommand.TargetMode.AUTO));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return AutoFile.buildAuto();
    }

    /** Hard-stop zero the intake pivot once when teleop starts. */
    public Command getTeleopIntakeZeroCommand() {
        return intaker.zeroCommand();
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Robot Voltage", RobotController.getBatteryVoltage());

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

        // Robot pose on the Field2d for Elastic (the path/target come from PathPlanner's
        // callbacks).
        FieldPublisher.setRobotPose(swerve.getEstimatedPose().toPose2d());
        AutoFile.updatePreview();

        // Vision ghost — re-homed out of the vendored lib so future lib copies stay drop-in. Logs
        // the
        // tag-derived robot pose as a Pose2d[], hidden (empty array) when there's no target so it
        // doesn't snap to the field origin. Bind a translucent robot to Vision/Ghost in
        // AdvantageScope.
        Pose2d visionPose = limelightSubsystem.getPose(LimeLightConfig.NAME);
        Logger.recordOutput(
                "Vision/Ghost",
                visionPose.equals(new Pose2d()) ? new Pose2d[0] : new Pose2d[] {visionPose});
    }

    public String getAutoSelectionSummary() {
        return AutoFile.selectionSummary();
    }

    /**
     * Safety net for {@link Robot#teleopInit}: autonomous's trench-start opening dash lifts the
     * swerve speed cap to unlimited and normally restores it before finishing, but if auto is
     * interrupted mid-dash (e.g. cancelled early) that restore step never runs. Force the default
     * limit back here so teleop never inherits an unlimited cap.
     */
    public void resetSwerveLimitForTeleop() {
        swerve.setSwerveLimitDefault();
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
                || intakeMode == IntakerConfig.IntakeMode.MAX_INTAKING
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
                isReal && RobotConstants.HAS_INTAKER_IO
                        ? new MotorIOTalonFX(IntakerConfig.INTAKER_ROLLER_CONFIG)
                        : new MotorIOSim(IntakerConfig.INTAKER_ROLLER_CONFIG),
                IntakerRollerParamsNT.asVelocityParamSources());
    }

    private PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> buildIntakerPivot() {
        return new PositionMotorSubsystem<>(
                IntakerConfig.INTAKER_PIVOT_CONFIG,
                new MotorInputsAutoLogged(),
                isReal && RobotConstants.HAS_INTAKER_IO
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
                isReal && RobotConstants.HAS_HOPPER_IO
                        ? new MotorIOTalonFX(HopperConfig.HOPPER_CONFIG)
                        : new MotorIOSim(HopperConfig.HOPPER_CONFIG));
    }

    private VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> buildShooterDrum() {
        return new VelocityMotorSubsystem<>(
                ShooterConfig.SHOOTER_DRUM_CONFIG,
                new MotorInputsAutoLogged(),
                isReal && RobotConstants.HAS_SHOOTER_IO
                        ? new MotorIOTalonFX(ShooterConfig.SHOOTER_DRUM_CONFIG)
                        : new MotorIOSim(ShooterConfig.SHOOTER_DRUM_CONFIG),
                ShooterUpperParamsNT.asVelocityParamSources());
    }

    private VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> buildShooterFeed() {
        return new VelocityMotorSubsystem<>(
                ShooterConfig.SHOOTER_FEED_CONFIG,
                new MotorInputsAutoLogged(),
                isReal && RobotConstants.HAS_SHOOTER_IO
                        ? new MotorIOTalonFX(ShooterConfig.SHOOTER_FEED_CONFIG)
                        : new MotorIOSim(ShooterConfig.SHOOTER_FEED_CONFIG),
                ShooterLowerParamsNT.asVelocityParamSources());
    }

    private LimelightSubsystem buildLimelight() {
        LimelightIOReal io =
                new LimelightIOReal(
                        LimeLightConfig.limelightConfig,
                        swerve::getIMUYaw,
                        swerve::getYawVelocityRadPerSec,
                        () -> false,
                        LimeLightConfig.asDeviationParams());
        return new LimelightSubsystem(swerve, io);
    }

    @SuppressWarnings("unused")
    private IndicatorSubsystem buildIndicator() {
        return new IndicatorSubsystem(
                isReal && RobotConstants.HAS_LED_IO
                        ? new IndicatorIOARGB(/* PWM port */ 9, /* LED count */ 60)
                        : new IndicatorIOSim());
    }

    private PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> buildHood() {
        return new PositionMotorSubsystem<>(
                ShooterConfig.HOOD_CONFIG,
                new MotorInputsAutoLogged(),
                isReal && RobotConstants.HAS_HOOD_IO
                        ? new MotorIOTalonFX(ShooterConfig.HOOD_CONFIG)
                        : new MotorIOSim(ShooterConfig.HOOD_CONFIG),
                HoodParamsNT.asPositionParamSources(),
                ShooterConfig.HOOD_STOW_ANGLE,
                ShooterConfig.HOOD_ANGLE_PER_ROTATION);
    }

    private Swerve buildSwerve() {
        if (isReal && HAS_SWERVE_IO) {
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
