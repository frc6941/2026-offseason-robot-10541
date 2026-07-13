package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStateRecorder;
import java.util.function.Supplier;
import lib.ironpulse.swerve.Swerve;
import lib.ntext.NTParameter;
import org.littletonrobotics.junction.Logger;

/**
 * Drives the swerve drivetrain to an {@link APTarget} using Autopilot.
 *
 * <p>Autopilot is a reactive point-to-point driver: each loop {@link Autopilot#calculate} returns a
 * field-relative translational velocity plus the heading the robot should hold. It does NOT command
 * an angular velocity, so this command runs its own heading PID to turn that target heading into
 * omega, then hands a robot-relative twist to {@link Swerve#runTwist}.
 *
 * <p>Translation/rotation tolerances and the velocity/accel/jerk limits live in the {@link
 * Autopilot}'s {@code APProfile}; this command only owns the heading controller.
 */
public class AutopilotDriveToPose extends Command {
    private static final String kTag = "Commands/AutopilotDriveToPose";

    private final Swerve swerve;
    private final Autopilot autopilot;
    private final Supplier<APTarget> targetSupplier;
    private final PIDController headingController = new PIDController(5.0, 0, 0);

    private APTarget target;

    public AutopilotDriveToPose(
            Swerve swerve, Autopilot autopilot, Supplier<APTarget> targetSupplier) {
        this.swerve = swerve;
        this.autopilot = autopilot;
        this.targetSupplier = targetSupplier;
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerve);
    }

    /** Convenience overload for a fixed target pose. */
    public AutopilotDriveToPose(Swerve swerve, Autopilot autopilot, APTarget target) {
        this(swerve, autopilot, () -> target);
    }

    @Override
    public void initialize() {
        target =
                targetSupplier
                        .get()
                        .withEntryAngle(
                                Rotation2d.fromDegrees(
                                        AutopilotDriveToPoseParamsNT.entryangle.getValue()));
        headingController.setP(AutopilotDriveToPoseParamsNT.headingKp.getValue());
        headingController.setI(AutopilotDriveToPoseParamsNT.headingKi.getValue());
        headingController.setD(AutopilotDriveToPoseParamsNT.headingKd.getValue());
        headingController.reset();
    }

    @Override
    public void execute() {
        Pose2d current = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
        ChassisSpeeds robotRelativeSpeeds = swerve.getChassisSpeeds();

        Autopilot.APResult result = autopilot.calculate(current, robotRelativeSpeeds, target);

        double vx = result.vx().in(MetersPerSecond);
        double vy = result.vy().in(MetersPerSecond);
        Rotation2d heading = current.getRotation();

        double omega =
                headingController.calculate(
                        heading.getRadians(), result.targetAngle().getRadians());

        // Autopilot emits field-relative vx/vy; runTwist expects robot-relative.
        swerve.runTwist(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, heading));

        Logger.recordOutput(kTag + "/targetPose", target.getReference());
        Logger.recordOutput(kTag + "/targetHeadingDeg", result.targetAngle().getDegrees());
        Logger.recordOutput(kTag + "/vxMps", vx);
        Logger.recordOutput(kTag + "/vyMps", vy);
        Logger.recordOutput(kTag + "/omegaRadPerSec", omega);
    }

    @Override
    public boolean isFinished() {
        return autopilot.atTarget(RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d(), target);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.runStop();
    }

    @NTParameter(tableName = "Params/" + kTag)
    public static class AutopilotDriveToPoseParams {
        static final double headingKp = 5.0;
        static final double headingKi = 0.0;
        static final double headingKd = 0.0;

        static final double entryangle = 90.0;
    }
}
