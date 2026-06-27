package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.utils.AllianceFlipUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/**
 * Rotates the swerve to face the Hub while still letting the driver translate.
 * Use this in Commands.parallel() alongside hoodSubsystem.runMotionMagic(angleSupplier)
 * to also adjust the hood angle simultaneously.
 */
public class AutoAimCommand extends Command {
    // Following 6328's constant
    private static final double DRIVE_LAUNCH_KP = 8.0;
    private static final double DRIVE_LAUNCH_KD = 0.0;
    private static final double MAX_ANGULAR_VEL_RAD_PER_SEC = 7.0;

    private final Swerve swerve;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final Supplier<Rotation2d> targetHeading;
    private final DoubleSupplier targetHeadingRate;

    public AutoAimCommand(Swerve swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> targetHeading, DoubleSupplier targetHeadingRate) {
        this.swerve = swerve;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.targetHeading = targetHeading;
        this.targetHeadingRate = targetHeadingRate;
        addRequirements(swerve);
    }

    

    // Drum shooter mounting relative to robot center, robot frame (+X fwd/intake, +Y left).
    // The rotation is the firing yaw: Math.PI = fires opposite the intake (out the back).
    // The translation's Y (lateral offset) drives the off-center aim correction.
    // TODO: set from CAD — drum position + firing yaw; tune live until the shooter faces the hub.
    public static final Transform2d ROBOT_TO_SHOOTER =
            new Transform2d(-0.15, 0.0, Rotation2d.fromRadians(Math.PI));

    public static Translation2d getTarget() {
        return AllianceFlipUtil.apply(FieldConstants.Hub.getTarget2d());
    }

    public static double getDistanceToTarget(Translation2d robotPos) {
        return getTarget().getDistance(robotPos);
    }

    /**
     * Chassis heading that points the SHOOTER (not robot-forward/intake) at the hub, à la 6328:
     * bearingToHub + asin(shooterLateralY / distance) + shooterFiringYaw.
     */
    public static Rotation2d getShooterAimHeading(Pose2d robotPose) {
        Translation2d target = getTarget();
        Rotation2d bearing = target.minus(robotPose.getTranslation()).getAngle();
        double distance = target.getDistance(robotPose.getTranslation());
        Rotation2d lateralCorrection = new Rotation2d(MathUtil.clamp(
                Math.asin(MathUtil.clamp(ROBOT_TO_SHOOTER.getY() / distance, -1.0, 1.0)),
                -Math.PI, Math.PI));
        return bearing.plus(lateralCorrection).plus(ROBOT_TO_SHOOTER.getRotation());
    }

    @Override
    public void execute() {
        var robotPose = swerve.getEstimatedPose().toPose2d();
        Translation2d toTarget = getTarget().minus(robotPose.getTranslation()); // get aiming vector

        Rotation2d target = targetHeading.get();
        double error = target.minus(robotPose.getRotation()).getRadians();
        double ffVel = targetHeadingRate.getAsDouble();
        double measureOmega = swerve.getYawVelocityRadPerSec();
        double omega = ffVel + DRIVE_LAUNCH_KP * error + DRIVE_LAUNCH_KD * (ffVel - measureOmega);
        omega = MathUtil.clamp(omega, -MAX_ANGULAR_VEL_RAD_PER_SEC, MAX_ANGULAR_VEL_RAD_PER_SEC);


        // Joystick translation — same convention as driveWithJoystick
        double maxSpeed = swerve.getSwerveLimit().maxLinearVelocity().in(MetersPerSecond);
        double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.1);
        double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), 0.1);
        double vNorm = Math.hypot(x, y) * maxSpeed;
        Translation2d v = new Translation2d(vNorm, new Rotation2d(x, y));
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                v.getX(), v.getY(), omega, robotPose.getRotation());
        swerve.runTwist(speeds);

        Logger.recordOutput("AutoAim/TargetHeading", target.getDegrees());
        Logger.recordOutput("AutoAim/Distance", toTarget.getNorm());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.runStop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
