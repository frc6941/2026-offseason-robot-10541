package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConstants;
import frc.robot.RobotStateRecorder;
import frc.robot.commands.AutoAimCommand;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;

public class ShooterSubsystem {
    private static final String MANUAL_OVERRIDE_KEY = "Shooter Tuning/Manual Override";
    private static final String MANUAL_HOOD_ANGLE_KEY = "Shooter Tuning/Hood Angle Deg";
    private static final String MANUAL_FLYWHEEL_RPS_KEY = "Shooter Tuning/Flywheel RPS";

    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> upper;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> lower;
    private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood;
    private final HopperSubsystem hopper;
    private final ShotCalculator calculator = new ShotCalculator();

    public ShooterSubsystem(
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> upper,
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> lower,
            PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood,
            HopperSubsystem hopper) {
        this.upper = upper;
        this.lower = lower;
        this.hood = hood;
        this.hopper = hopper;

        SmartDashboard.setDefaultBoolean(MANUAL_OVERRIDE_KEY, false);
        SmartDashboard.setDefaultNumber(MANUAL_HOOD_ANGLE_KEY, 10.0);
        SmartDashboard.setDefaultNumber(MANUAL_FLYWHEEL_RPS_KEY, 55.0);
        if (!RobotConstants.ENABLE_NT_PARAMS) {
            SmartDashboard.putBoolean(MANUAL_OVERRIDE_KEY, false);
        }
    }

    public void configureDefaultCommands() {
        upper.setDefaultCommand(
                upper.runVelVolt(
                        () -> RotationsPerSecond.of(ShooterUpperParamsNT.idleRPS.getValue())));
        lower.setDefaultCommand(
                lower.runVelVolt(
                        () -> RotationsPerSecond.of(ShooterLowerParamsNT.idleRPS.getValue())));
        hood.setDefaultCommand(hood.runMotionMagic(ShooterConfig.HOOD_STOW_ANGLE));
    }

    public Command shoot(BooleanSupplier forceMaxHood) {
        return shoot(
                () ->
                        AutoAimCommand.getDistanceToTarget(
                                RobotStateRecorder.getPoseWorldRobotCurrent()
                                        .getTranslation()
                                        .toTranslation2d()),
                forceMaxHood);
    }

    public Command shootAtDistance(DoubleSupplier distanceMeters, BooleanSupplier forceMaxHood) {
        return shoot(distanceMeters, forceMaxHood);
    }

    private Command shoot(DoubleSupplier distanceMeters, BooleanSupplier forceMaxHood) {
        Command waitForUpper =
                Commands.waitUntil(() -> upperReadyAtDistance(distanceMeters.getAsDouble()));
        return Commands.parallel(
                upper.runVelVolt(() -> solution(distanceMeters.getAsDouble()).shooterSpeed()),
                lower.runVelVolt(
                                () ->
                                        RotationsPerSecond.of(
                                                ShooterLowerParamsNT.idleRPS.getValue()))
                        .until(() -> upperReadyAtDistance(distanceMeters.getAsDouble()))
                        .andThen(
                                lower.runVelVolt(
                                        () ->
                                                RotationsPerSecond.of(
                                                        solution(distanceMeters.getAsDouble())
                                                                        .shooterSpeed()
                                                                        .in(RotationsPerSecond)
                                                                * calculator
                                                                        .lowerShooterSpeedScale()))),
                hood.runMotionMagic(
                        () ->
                                forceMaxHood.getAsBoolean()
                                        ? ShooterConfig.HOOD_MAX_ANGLE
                                        : clampHoodAngle(
                                                solution(distanceMeters.getAsDouble())
                                                        .hoodAngle())),
                Commands.sequence(Commands.deadline(waitForUpper, hopper.idle()), hopper.shoot()));
    }

    public Command spinUp() {
        return Commands.parallel(
                upper.runVelVolt(() -> currentSolution().shooterSpeed()),
                hood.runMotionMagic(() -> clampHoodAngle(currentSolution().hoodAngle())));
    }

    public Command idle() {
        return Commands.parallel(
                upper.runVelVolt(
                        () -> RotationsPerSecond.of(ShooterUpperParamsNT.idleRPS.getValue())),
                lower.runVelVolt(
                        () -> RotationsPerSecond.of(ShooterLowerParamsNT.idleRPS.getValue())),
                hood.runMotionMagic(ShooterConfig.HOOD_STOW_ANGLE),
                hopper.idle());
    }

    public Command stop() {
        return upper.runVelVolt(
                () -> RotationsPerSecond.of(ShooterUpperParamsNT.stopRPS.getValue()));
    }

    public Command zeroHood() {
        return hood.zeroCommand().withTimeout(ShooterConfig.HOOD_ZEROING_TIMEOUT_SECONDS);
    }

    public boolean upperReady() {
        return upperReadyAtDistance(currentDistance());
    }

    public boolean upperReadyAtDistance(double distanceMeters) {
        return upper.getVelocity()
                .isNear(
                        solution(distanceMeters).shooterSpeed(),
                        RotationsPerSecond.of(
                                ShooterUpperParamsNT.velocityAtGoalToleranceRPS.getValue()));
    }

    public ShotSolution currentSolution() {
        return solution(currentDistance());
    }

    public ShotSolution solution(double distanceMeters) {
        if (!manualOverrideEnabled()) {
            return calculator.solve(distanceMeters);
        }
        return new ShotSolution(
                clampHoodAngle(Degrees.of(SmartDashboard.getNumber(MANUAL_HOOD_ANGLE_KEY, 10.0))),
                RotationsPerSecond.of(
                        Math.max(0.0, SmartDashboard.getNumber(MANUAL_FLYWHEEL_RPS_KEY, 55.0))));
    }

    public double lowerSpeedRps(ShotSolution solution) {
        return solution.shooterSpeed().in(RotationsPerSecond) * calculator.lowerShooterSpeedScale();
    }

    public double headingToleranceDegrees() {
        return calculator.headingToleranceDeg();
    }

    public boolean manualOverrideEnabled() {
        return RobotConstants.ENABLE_NT_PARAMS
                && SmartDashboard.getBoolean(MANUAL_OVERRIDE_KEY, false);
    }

    private double currentDistance() {
        return AutoAimCommand.getDistanceToTarget(
                RobotStateRecorder.getPoseWorldRobotCurrent().getTranslation().toTranslation2d());
    }

    private Angle clampHoodAngle(Angle angle) {
        return Degrees.of(
                MathUtil.clamp(
                        angle.in(Degrees),
                        ShooterConfig.HOOD_MIN_ANGLE.in(Degrees),
                        ShooterConfig.HOOD_MAX_ANGLE.in(Degrees)));
    }
}
