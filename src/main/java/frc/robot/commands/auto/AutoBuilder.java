package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStateRecorder;
import frc.robot.subsystems.Configs.SwerveMK5Config;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.subsystems.Shooter.ShootingSuperstructure;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.utils.AllianceFlipUtil;

public final class AutoBuilder {
    private static boolean configured = false;

    private final IntakerSubsystem intaker;
    private final Swerve swerve;
    private final ShootingSuperstructure shootingSuperstructure;

    public AutoBuilder(
            IntakerSubsystem intaker,
            Swerve swerve,
            ShootingSuperstructure shootingSuperstructure) {
        this.intaker = intaker;
        this.swerve = swerve;
        this.shootingSuperstructure = shootingSuperstructure;
    }

    public Pose2d getCurrentPose() {
        return RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
    }

    public static void configure(Swerve swerve) {
        if (configured) {
            return;
        }

        com.pathplanner.lib.auto.AutoBuilder.configure(
                () -> swerve.getEstimatedPose().toPose2d(),
                pose -> swerve.resetEstimatedPose(new Pose3d(pose)),
                swerve::getChassisSpeeds,
                (ChassisSpeeds speeds) -> swerve.runTwist(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                createRobotConfig(),
                AllianceFlipUtil::shouldFlip,
                swerve);

        configured = true;
    }

    public static RobotConfig createRobotConfig() {
        Translation2d[] moduleLocations = {
            new Translation2d(SwerveMK5Config.kSwerveHalfLength, SwerveMK5Config.kSwerveHalfWidth),
            new Translation2d(SwerveMK5Config.kSwerveHalfLength, -SwerveMK5Config.kSwerveHalfWidth),
            new Translation2d(-SwerveMK5Config.kSwerveHalfLength, SwerveMK5Config.kSwerveHalfWidth),
            new Translation2d(-SwerveMK5Config.kSwerveHalfLength, -SwerveMK5Config.kSwerveHalfWidth)
        };

        ModuleConfig moduleConfig =
                new ModuleConfig(
                        Inch.of(4.0).div(2.0),
                        InchesPerSecond.of(5800 / 60.0 / 7.03 * Math.PI * 4.0),
                        1.0,
                        DCMotor.getKrakenX60Foc(1),
                        7.03,
                        Amps.of(65),
                        1);

        return new RobotConfig(
                Kilograms.of(52), KilogramSquareMeters.of(0.04), moduleConfig, moduleLocations);
    }

    public Command buildDepotXAuto() {
        return AutoCommands.depotXCollect(swerve, intaker);
    }

    public Command buildDepotYAuto() {
        return AutoCommands.depotYCollect(swerve, intaker);
    }

    public Command buildOutpostAuto() {
        return AutoCommands.goToOutpost(swerve);
    }

    public Command buildHubCenterStartAuto() {
        return AutoCommands.goToHubCenterStart(swerve);
    }

    public Command buildMidSweepLeftToRightAuto() {
        return AutoCommands.sweepMidLeftToRight(swerve, intaker);
    }

    public Command buildMidSweepRightToLeftAuto() {
        return AutoCommands.sweepMidRightToLeft(swerve, intaker);
    }

    public Command buildNeutralSweepAuto(
            AutoCommands.NeutralSweepMode mode, AutoCommands.NeutralSweepDirection direction) {
        return AutoCommands.neutralZoneSweep(swerve, intaker, mode, direction);
    }

    public Command buildNeutralSweepAuto(
            AutoCommands.NeutralSweepMode mode,
            AutoCommands.NeutralSweepDirection direction,
            AutoCommands.MidKind kind) {
        return AutoCommands.neutralZoneSweep(swerve, intaker, mode, direction, kind);
    }

    public Command buildMidTwoCycleAuto(
            AutoCommands.NeutralSweepMode firstMode,
            AutoCommands.NeutralSweepMode secondMode,
            AutoCommands.NeutralSweepDirection firstDirection,
            AutoCommands.NeutralSweepDirection secondDirection,
            AutoCommands.MidKind firstKind,
            AutoCommands.MidKind secondKind,
            AutoSelector.Side firstShootPosition,
            AutoSelector.Side secondShootPosition,
            AutoSelector.DepotAxis depotAxis,
            AutoCommands.DepotVisitRound depotRound,
            AutoCommands.DepotVisitRound outpostRound) {
        return AutoCommands.midTwoCycle(
                swerve,
                intaker,
                shootingSuperstructure,
                firstMode,
                secondMode,
                firstDirection,
                secondDirection,
                firstKind,
                secondKind,
                firstShootPosition,
                secondShootPosition,
                depotAxis,
                depotRound,
                outpostRound);
    }

    public Command buildLeftTrenchClearAuto() {
        return AutoCommands.trenchLeftStartToClear(swerve);
    }

    public Command buildRightTrenchClearAuto() {
        return AutoCommands.trenchRightStartToClear(swerve);
    }

    public Command buildLeftBumpCrossAuto() {
        return AutoCommands.bumpLeftInnerToOuter(swerve);
    }

    public Command buildRightBumpCrossAuto() {
        return AutoCommands.bumpRightInnerToOuter(swerve);
    }

    public Command buildDepotLeftThroughAuto() {
        return AutoCommands.depotLeftThrough(swerve, intaker);
    }

    public Command buildDepotRightThroughAuto() {
        return AutoCommands.depotRightThrough(swerve, intaker);
    }

    public Command buildLeftBumpLaunchAuto() {
        return AutoCommands.goToLeftBumpLaunch(swerve);
    }

    public Command buildRightBumpLaunchAuto() {
        return AutoCommands.goToRightBumpLaunch(swerve);
    }

    public Command buildLeftTrenchLaunchAuto() {
        return AutoCommands.goToLeftTrenchLaunch(swerve);
    }

    public Command buildRightTrenchLaunchAuto() {
        return AutoCommands.goToRightTrenchLaunch(swerve);
    }

    public Command buildLeftClimbAuto() {
        return AutoCommands.goToLeftClimb(swerve);
    }

    public Command buildRightClimbAuto() {
        return AutoCommands.goToRightClimb(swerve);
    }

    public Command buildLeftTowerThroughAuto() {
        return AutoCommands.towerLeftThrough(swerve);
    }

    public Command buildRightTowerThroughAuto() {
        return AutoCommands.towerRightThrough(swerve);
    }
}
