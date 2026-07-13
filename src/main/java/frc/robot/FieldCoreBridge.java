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
import frc.robot.subsystems.Hopper.HopperConfig;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import frc.robot.subsystems.Intaker.IntakerConfig.IntakeMode;
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
        Pose3d pose = swerve.getEstimatedPose();
        ChassisSpeeds chassisSpeeds = swerve.getChassisSpeeds();
        SwerveModuleState[] moduleStates = swerve.getModuleStates();
        boolean intakeEnabled = isIntakeEnabled();
        boolean shooterActive = isShooterActive();
        boolean feedActive = isFeedActive();
        boolean shootCommand = shooterActive && feedActive;
        if (shootCommand && !previousShootCommand) {
            shootCount++;
        }
        previousShootCommand = shootCommand;

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
        table.getEntry("ShooterRPM")
                .setDouble(shooterUpper.getCurrSetpoint().in(RotationsPerSecond) * 60.0);
        table.getEntry("ShooterUpperRPM")
                .setDouble(shooterUpper.getCurrSetpoint().in(RotationsPerSecond) * 60.0);
        table.getEntry("ShooterLowerRPM")
                .setDouble(shooterLower.getCurrSetpoint().in(RotationsPerSecond) * 60.0);
        table.getEntry("HoodAngleDeg").setDouble(hood.getCurrPos().in(Degrees));
        table.getEntry("HoodTargetDeg").setDouble(hood.getCurrSetpoint().in(Degrees));
        table.getEntry("ShootCommand").setBoolean(shootCommand);
        table.getEntry("ShootCount").setDouble(shootCount);

        Logger.recordOutput("FieldCoreBridge/IntakeEnabled", intakeEnabled);
        Logger.recordOutput("FieldCoreBridge/FeedActive", feedActive);
        Logger.recordOutput("FieldCoreBridge/ShooterEnabled", shooterActive);
        Logger.recordOutput(
                "FieldCoreBridge/ShooterUpperRPM",
                shooterUpper.getCurrSetpoint().in(RotationsPerSecond) * 60.0);
        Logger.recordOutput(
                "FieldCoreBridge/ShooterLowerRPM",
                shooterLower.getCurrSetpoint().in(RotationsPerSecond) * 60.0);
        Logger.recordOutput("FieldCoreBridge/HoodAngleDeg", hood.getCurrPos().in(Degrees));
        Logger.recordOutput("FieldCoreBridge/HoodTargetDeg", hood.getCurrSetpoint().in(Degrees));
        Logger.recordOutput("FieldCoreBridge/ShootCommand", shootCommand);
        Logger.recordOutput("FieldCoreBridge/ShootCount", shootCount);
        Logger.recordOutput("FieldCoreBridge/ModuleStates", moduleStates);
    }

    private boolean isIntakeEnabled() {
        IntakeMode mode = intaker.getCurrentMode();
        return mode == IntakeMode.INTAKING
                || mode == IntakeMode.FEEDING
                || mode == IntakeMode.EXTENDED_REVERSE
                || mode == IntakeMode.RETRACTED_FEEDING;
    }

    private boolean isShooterActive() {
        return shooterUpper.getCurrSetpoint().in(RotationsPerSecond)
                        > ShooterUpperParamsNT.idleRPS.getValue() + 1.0
                || shooterLower.getCurrSetpoint().in(RotationsPerSecond)
                        > ShooterLowerParamsNT.idleRPS.getValue() + 1.0;
    }

    private boolean isFeedActive() {
        return hopper.getCurrSetpoint().in(RotationsPerSecond)
                > HopperConfig.HopperParams.idleRPS + 0.05;
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
