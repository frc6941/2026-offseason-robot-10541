package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Hopper.HopperParamsNT;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.subsystems.Shooter.ShooterLowerParamsNT;
import frc.robot.subsystems.Shooter.ShooterUpperParamsNT;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lib.ironpulse.swerve.Swerve;
import org.littletonrobotics.junction.Logger;

/** Publishes this robot project's simulation state to FieldCore. */
public class FieldCoreBridge extends SubsystemBase {
    private final Swerve swerve;
    private final IntakerSubsystem intaker;
    private final HopperSubsystem hopper;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooterUpper;
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooterLower;
    private final PositionMotorSubsystem<
                    MotorInputsAutoLogged, MotorIO, edu.wpi.first.units.measure.Angle>
            hood;
    private final NetworkTable table;

    private boolean previousShootCommand = false;
    private int shootCount = 0;

    public FieldCoreBridge(
            Swerve swerve,
            IntakerSubsystem intaker,
            HopperSubsystem hopper,
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooterUpper,
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooterLower,
            PositionMotorSubsystem<
                            MotorInputsAutoLogged, MotorIO, edu.wpi.first.units.measure.Angle>
                    hood) {
        super("FieldCoreBridge");
        this.swerve = swerve;
        this.intaker = intaker;
        this.hopper = hopper;
        this.shooterUpper = shooterUpper;
        this.shooterLower = shooterLower;
        this.hood = hood;
        this.table = NetworkTableInstance.getDefault().getTable("FieldCore/Robot");
    }

    @Override
    public void periodic() {
        Pose3d pose = RobotStateRecorder.getPoseWorldRobotCurrent();
        ChassisSpeeds chassisSpeeds = RobotStateRecorder.getChassisSpeeds();
        SwerveModuleState[] moduleStates = swerve.getModuleStates();
        boolean intakeEnabled = isIntakeEnabled();
        boolean shooterActive = isShooterActive();
        boolean feedActive = isFeedActive();
        boolean shootCommand = shooterActive && feedActive;
        if (shootCommand && !previousShootCommand) {
            shootCount++;
        }
        previousShootCommand = shootCommand;

        // Snapshot each setpoint/position once — they feed both the NT table and the AdvantageKit
        // log below, and don't change within a loop.
        double shooterUpperRPM = shooterUpper.getCurrSetpoint().in(RotationsPerSecond) * 60.0;
        double shooterLowerRPM = shooterLower.getCurrSetpoint().in(RotationsPerSecond) * 60.0;
        double hoodAngleDeg = hood.getCurrPos().in(Degrees);
        double hoodTargetDeg = hood.getCurrSetpoint().in(Degrees);

        table.getEntry("Enabled").setBoolean(DriverStation.isEnabled());
        table.getEntry("PoseEstimate").setDoubleArray(poseToArray(pose));
        table.getEntry("ChassisSpeeds")
                .setDoubleArray(
                        new double[] {
                            chassisSpeeds.vxMetersPerSecond,
                            chassisSpeeds.vyMetersPerSecond,
                            chassisSpeeds.omegaRadiansPerSecond
                        });
        table.getEntry("ModuleStates").setDoubleArray(moduleStatesToArray(moduleStates));
        table.getEntry("IntakeEnabled").setBoolean(intakeEnabled);
        table.getEntry("ShooterEnabled").setBoolean(shooterActive);
        table.getEntry("ShooterRPM").setDouble(shooterUpperRPM);
        table.getEntry("ShooterUpperRPM").setDouble(shooterUpperRPM);
        table.getEntry("ShooterLowerRPM").setDouble(shooterLowerRPM);
        table.getEntry("HoodAngleDeg").setDouble(hoodAngleDeg);
        table.getEntry("HoodTargetDeg").setDouble(hoodTargetDeg);
        table.getEntry("ShootCommand").setBoolean(shootCommand);
        table.getEntry("ShootCount").setDouble(shootCount);

        Logger.recordOutput("FieldCoreBridge/IntakeEnabled", intakeEnabled);
        Logger.recordOutput("FieldCoreBridge/FeedActive", feedActive);
        Logger.recordOutput("FieldCoreBridge/ShooterEnabled", shooterActive);
        Logger.recordOutput("FieldCoreBridge/ShooterUpperRPM", shooterUpperRPM);
        Logger.recordOutput("FieldCoreBridge/ShooterLowerRPM", shooterLowerRPM);
        Logger.recordOutput("FieldCoreBridge/HoodAngleDeg", hoodAngleDeg);
        Logger.recordOutput("FieldCoreBridge/HoodTargetDeg", hoodTargetDeg);
        Logger.recordOutput("FieldCoreBridge/ShootCommand", shootCommand);
        Logger.recordOutput("FieldCoreBridge/ShootCount", shootCount);
        Logger.recordOutput("FieldCoreBridge/ModuleStates", moduleStates);
    }

    private boolean isIntakeEnabled() {
        return intaker.isActive();
    }

    private boolean isShooterActive() {
        return shooterUpper.getCurrSetpoint().in(RotationsPerSecond)
                        > ShooterUpperParamsNT.idleRPS.getValue() + 1.0
                || shooterLower.getCurrSetpoint().in(RotationsPerSecond)
                        > ShooterLowerParamsNT.idleRPS.getValue() + 1.0;
    }

    private boolean isFeedActive() {
        return hopper.getCurrSetpoint().in(RotationsPerSecond)
                > HopperParamsNT.idleRPS.getValue() + 0.05;
    }

    private static double[] poseToArray(Pose3d pose) {
        return new double[] {
            pose.getX(),
            pose.getY(),
            pose.getZ(),
            pose.getRotation().getX(),
            pose.getRotation().getY(),
            pose.getRotation().getZ()
        };
    }

    private static double[] moduleStatesToArray(SwerveModuleState[] states) {
        double[] values = new double[states.length * 2];
        for (int i = 0; i < states.length; i++) {
            values[i * 2] = states[i].speedMetersPerSecond;
            values[i * 2 + 1] = states[i].angle.getRadians();
        }
        return values;
    }
}
