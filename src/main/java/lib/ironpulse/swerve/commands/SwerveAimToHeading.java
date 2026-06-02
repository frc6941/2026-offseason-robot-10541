package lib.ironpulse.swerve.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static lib.ironpulse.math.MathTools.epsilonEquals;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import lib.ironpulse.swerve.Swerve;
import lib.ntext.NTParameter;
import org.littletonrobotics.junction.Logger;

public class SwerveAimToHeading extends Command {
    private static final String kTag = "Commands/AimToHeading";
    private final Swerve swerve;
    protected Supplier<Pose3d> poseWorldRobotSupplier;
    protected Supplier<Rotation2d> headingWorldTargetSupplier;
    protected Angle rotationTolerance;
    protected PIDController rotationController;
    protected Supplier<AngularVelocity> rotationVelocityMaxSupplier;

    public SwerveAimToHeading(
            Swerve swerve,
            Supplier<Pose3d> poseWorldRobotSupplier,
            Supplier<Rotation2d> headingWorldTargetSupplier,
            PIDController rotationController,
            Angle rotationTolerance,
            Supplier<AngularVelocity> rotationVelocityMaxSupplier) {
        this.swerve = swerve;
        this.poseWorldRobotSupplier = poseWorldRobotSupplier;
        this.headingWorldTargetSupplier = headingWorldTargetSupplier;
        this.rotationController = rotationController;
        this.rotationTolerance = rotationTolerance;
        this.rotationVelocityMaxSupplier = rotationVelocityMaxSupplier;
        addRequirements(swerve);

        rotationController.enableContinuousInput(0, Math.PI * 2);
    }

    @Override
    public void initialize() {
        rotationController.setP(SwerveAimToHeadingParamsNT.rotationKp.getValue());
        rotationController.setI(SwerveAimToHeadingParamsNT.rotationKi.getValue());
        rotationController.setD(SwerveAimToHeadingParamsNT.rotationKd.getValue());

        rotationController.reset();
    }

    @Override
    public void execute() {
        // get from supplier
        Pose3d TWR = poseWorldRobotSupplier.get();
        Rotation2d headingWT = headingWorldTargetSupplier.get();

        // compute rotation error and angular velocity
        Rotation2d headingWR = TWR.getRotation().toRotation2d();
        double thetaRT = headingWT.minus(headingWR).getRadians();
        double omegaRT = -rotationController.calculate(thetaRT, 0.0);
        double rotationVelocityMax = rotationVelocityMaxSupplier.get().in(RadiansPerSecond);
        omegaRT = MathUtil.clamp(omegaRT, -rotationVelocityMax, rotationVelocityMax);

        // compose and run velocity
        ChassisSpeeds VRT = new ChassisSpeeds(0.0, 0.0, omegaRT);
        swerve.runTwist(VRT);

        // logging
        Logger.recordOutput(kTag + "/headingWorldTarget", headingWT.getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.runStop();
    }

    @Override
    public boolean isFinished() {
        Pose3d TWR = poseWorldRobotSupplier.get();
        Rotation2d headingWT = headingWorldTargetSupplier.get();

        double tolRotDeg = rotationTolerance.in(Degrees);

        Rotation2d headingWR = TWR.getRotation().toRotation2d();
        boolean rotationOnTarget =
                epsilonEquals(headingWT.minus(headingWR).getDegrees(), 0.0, tolRotDeg);
        Logger.recordOutput(kTag + "/rotationOnTarget", rotationOnTarget);
        return rotationOnTarget;
    }

    @NTParameter(tableName = "Params/" + kTag)
    public static class SwerveAimToHeadingParams {
        static final double rotationKp = 5.0;
        static final double rotationKi = 0.0;
        static final double rotationKd = 0.0;
        static final double rotationVelocityMax = 5.0;
    }
}
