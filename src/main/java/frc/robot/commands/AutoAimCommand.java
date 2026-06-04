package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.utils.AllianceFlipUtil;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Rotates the swerve to face the Hub while still letting the driver translate.
 * Use this in Commands.parallel() alongside hoodSubsystem.runMotionMagic(angleSupplier)
 * to also adjust the hood angle simultaneously.
 */
public class AutoAimCommand extends Command {
    private static final double ROTATION_KP = 5.0;
    private static final double ROTATION_KI = 0.0;
    private static final double ROTATION_KD = 0.0;
    private static final double MAX_OMEGA_RAD_PER_SEC = 4.0;

    private final Swerve swerve;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final PIDController rotController;

    public AutoAimCommand(Swerve swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this.swerve = swerve;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotController = new PIDController(ROTATION_KP, ROTATION_KI, ROTATION_KD);
        rotController.enableContinuousInput(-Math.PI, Math.PI); // so that -pi and pi are the same angle
        addRequirements(swerve); // occupies swerve during command
    }

    public static Translation2d getTarget() {
        return AllianceFlipUtil.apply(FieldConstants.Hub.getTarget2d());
    }

    public static double getDistanceToTarget(Translation2d robotPos) {
        return getTarget().getDistance(robotPos);
    }

    @Override
    public void execute() {
        var robotPose = swerve.getEstimatedPose().toPose2d();
        Translation2d toTarget = getTarget().minus(robotPose.getTranslation()); // get aiming vector

        // Heading from robot to hub
        Rotation2d targetHeading = toTarget.getAngle(); // heading angle
        double omega = rotController.calculate(
                robotPose.getRotation().getRadians(),
                targetHeading.getRadians());
        omega = MathUtil.clamp(omega, -MAX_OMEGA_RAD_PER_SEC, MAX_OMEGA_RAD_PER_SEC);

        // Joystick translation — same convention as driveWithJoystick
        double maxSpeed = swerve.getSwerveLimit().maxLinearVelocity().in(MetersPerSecond);
        double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.1);
        double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), 0.1);
        double vNorm = Math.hypot(x, y) * maxSpeed;
        Translation2d v = new Translation2d(vNorm, new Rotation2d(x, y));
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                v.getX(), v.getY(), omega, robotPose.getRotation());
        swerve.runTwist(speeds);

        Logger.recordOutput("AutoAim/TargetHeading", targetHeading.getDegrees());
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
