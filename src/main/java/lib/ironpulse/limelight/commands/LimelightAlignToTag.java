package lib.ironpulse.limelight.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import lib.ironpulse.limelight.LimelightIOInputsAutoLogged;
import lib.ironpulse.limelight.LimelightSubsystem;
import lib.ironpulse.swerve.Swerve;
import lib.ntext.NTParameter;
import org.littletonrobotics.junction.Logger;

/**
 * Closed-loop alignment to an AprilTag using the Limelight's {@code targetpose_robotspace} data.
 *
 * <p>Unlike the Kalman-filter-based global correction (slow, field-frame), this command drives
 * directly against the robot→tag transform — ~1ms latency from the targeting pipeline. Use this
 * for precision scoring alignment (reef, barge) where sub-centimeter accuracy matters.
 *
 * <p>Typical usage — hold a button to align to tag 7 at a 1-meter standoff:
 * <pre>{@code
 * driverController.x().whileTrue(new LimelightAlignToTag(
 *     swerve, limelight, "limelight", 7,
 *     new Transform2d(1.0, 0.0, Rotation2d.kPi)));  // face the tag
 * }</pre>
 */
public class LimelightAlignToTag extends Command {
    private static final String kTag = "Commands/LimelightAlignToTag";

    private final Swerve swerve;
    private final LimelightSubsystem limelight;
    private final String limelightName;
    private final int targetTagId;
    private final Transform2d desiredOffset;

    private final PIDController xController = new PIDController(0, 0, 0);
    private final PIDController yController = new PIDController(0, 0, 0);
    private final PIDController thetaController = new PIDController(0, 0, 0);

    private static final double MAX_ROTATION_SPEED = 3.0;    // rad/s

    /**
     * @param swerve        the swerve subsystem
     * @param limelight     the limelight subsystem
     * @param limelightName which limelight to read (e.g. "limelight")
     * @param targetTagId   the AprilTag ID to align to, or -1 to align to whatever tag is visible
     * @param desiredOffset desired robot pose relative to the tag (robot→tag transform)
     */
    public LimelightAlignToTag(
            Swerve swerve,
            LimelightSubsystem limelight,
            String limelightName,
            int targetTagId,
            Transform2d desiredOffset) {
        this.swerve = swerve;
        this.limelight = limelight;
        this.limelightName = limelightName;
        this.targetTagId = targetTagId;
        this.desiredOffset = desiredOffset;
        addRequirements(swerve);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        xController.setPID(
                AlignToTagParamsNT.translationKp.getValue(),
                AlignToTagParamsNT.translationKi.getValue(),
                AlignToTagParamsNT.translationKd.getValue());
        yController.setPID(
                AlignToTagParamsNT.translationKp.getValue(),
                AlignToTagParamsNT.translationKi.getValue(),
                AlignToTagParamsNT.translationKd.getValue());
        thetaController.setPID(
                AlignToTagParamsNT.rotationKp.getValue(),
                AlignToTagParamsNT.rotationKi.getValue(),
                AlignToTagParamsNT.rotationKd.getValue());

        xController.reset();
        yController.reset();
        thetaController.reset();
    }

    @Override
    public void execute() {
        LimelightIOInputsAutoLogged inputs = limelight.getInputs(limelightName);

        // No valid target, or wrong tag (skip filter when targetTagId < 0) → coast
        boolean wrongTag = targetTagId >= 0 && (int) inputs.tid != targetTagId;
        if (!inputs.tv || wrongTag || inputs.targetPoseRobotSpace == null) {
            Logger.recordOutput(kTag + "/hasTarget", false);
            return;
        }
        Logger.recordOutput(kTag + "/hasTarget", true);

        // targetPoseRobotSpace gives the tag's position in the robot frame:
        //   +X = tag is ahead of robot
        //   +Y = tag is to the left of robot
        //   yaw = tag's rotation relative to robot
        Pose3d targetInRobot = inputs.targetPoseRobotSpace;
        double tagX = targetInRobot.getX();  // how far ahead the tag is
        double tagY = targetInRobot.getY();  // how far left the tag is
        double tagYaw = targetInRobot.getRotation().toRotation2d().getRadians();

        // Error = current robot→tag minus desired robot→tag
        // The desired offset says "I want the tag to appear at these (x,y,yaw) in robot frame"
        double errorX = tagX - desiredOffset.getX();
        double errorY = tagY - desiredOffset.getY();
        double errorTheta = tagYaw - desiredOffset.getRotation().getRadians();

        // Normalize theta to [-π, π]
        errorTheta = MathUtil.angleModulus(errorTheta);

        // PID on each axis
        double vx = -xController.calculate(errorX, 0.0);
        double vy = -yController.calculate(errorY, 0.0);
        double omega = -thetaController.calculate(errorTheta, 0.0);

        // Clamp speeds for safety
        double maxSpeed = swerve.getSwerveLimit().maxLinearVelocity().in(MetersPerSecond);
        double vxClamped = MathUtil.clamp(vx, -maxSpeed, maxSpeed);
        double vyClamped = MathUtil.clamp(vy, -maxSpeed, maxSpeed);
        double omegaClamped = MathUtil.clamp(omega, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

        swerve.runTwist(new ChassisSpeeds(vxClamped, vyClamped, omegaClamped));

        Logger.recordOutput(kTag + "/errorX", errorX);
        Logger.recordOutput(kTag + "/errorY", errorY);
        Logger.recordOutput(kTag + "/errorThetaDeg", Math.toDegrees(errorTheta));
        Logger.recordOutput(kTag + "/vx", vxClamped);
        Logger.recordOutput(kTag + "/vy", vyClamped);
        Logger.recordOutput(kTag + "/omega", omegaClamped);
        Logger.recordOutput(kTag + "/targetTagId", (int) inputs.tid);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.runStop();
        Logger.recordOutput(kTag + "/hasTarget", false);
    }

    @Override
    public boolean isFinished() {
        return false; // run while held
    }

    @NTParameter(tableName = "Params/" + kTag)
    public static class AlignToTagParams {
        static final double translationKp = 2.0;
        static final double translationKi = 0.0;
        static final double translationKd = 0.0;

        static final double rotationKp = 5.0;
        static final double rotationKi = 0.0;
        static final double rotationKd = 0.0;
    }
}
