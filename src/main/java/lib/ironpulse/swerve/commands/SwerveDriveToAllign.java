package lib.ironpulse.swerve.commands;

import static lib.ironpulse.math.MathTools.epsilonEquals;
import static lib.ironpulse.math.MathTools.toAngle;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveLimit;
import lib.ntext.NTParameter;
import org.littletonrobotics.junction.Logger;

public class SwerveDriveToAllign extends Command {
    private static final String kTag = "Commands/DriveToAllign";
    private static final double kDefaultDtSeconds = 0.02;
    private static final double kShiftEps = 1e-6;

    private final Swerve swerve;
    private final Supplier<Pose3d> poseWorldRobotSupplier;
    private final Supplier<Pose2d> velocityWorldRobotSupplier;
    private final Supplier<Pose2d> finalDestinationSupplier;
    private final Supplier<Rotation2d> shiftingDirectionSupplier;
    private final PIDController translationController;
    private final PIDController rotationController;
    private final SwerveLimit precisePointLimit;
    private final double parallelShiftMaxMeters;
    private final double lateralShiftMaxMeters;

    private Pose2d finalDestination = Pose2d.kZero;
    private Pose2d driveTarget = Pose2d.kZero;
    private Pose2d velocityWorldRobot = Pose2d.kZero;

    public SwerveDriveToAllign(
            Swerve swerve,
            Supplier<Pose3d> poseWorldRobotSupplier,
            Supplier<Pose2d> velocityWorldRobotSupplier,
            Supplier<Pose2d> finalDestinationSupplier,
            Supplier<Rotation2d> shiftingDirectionSupplier,
            SwerveLimit precisePointLimit,
            double parallelShiftMaxMeters,
            double lateralShiftMaxMeters) {
        this.swerve = swerve;
        this.poseWorldRobotSupplier = poseWorldRobotSupplier;
        this.velocityWorldRobotSupplier = velocityWorldRobotSupplier;
        this.finalDestinationSupplier = finalDestinationSupplier;
        this.shiftingDirectionSupplier = shiftingDirectionSupplier;
        this.translationController = new PIDController(0.0, 0.0, 0.0);
        this.rotationController = new PIDController(0.0, 0.0, 0.0);
        this.precisePointLimit = precisePointLimit;
        this.parallelShiftMaxMeters = Math.max(0.0, parallelShiftMaxMeters);
        this.lateralShiftMaxMeters = Math.max(0.0, lateralShiftMaxMeters);
        addRequirements(swerve);

        this.rotationController.enableContinuousInput(0, Math.PI * 2);
    }

    @Override
    public void initialize() {
        translationController.setP(SwerveDriveToAllignParamsNT.translationKp.getValue());
        translationController.setI(SwerveDriveToAllignParamsNT.translationKi.getValue());
        translationController.setIZone(SwerveDriveToAllignParamsNT.translationKiZone.getValue());
        translationController.setD(SwerveDriveToAllignParamsNT.translationKd.getValue());

        rotationController.setP(SwerveDriveToAllignParamsNT.rotationKp.getValue());
        rotationController.setI(SwerveDriveToAllignParamsNT.rotationKi.getValue());
        rotationController.setIZone(SwerveDriveToAllignParamsNT.rotationKiZone.getValue());
        rotationController.setD(SwerveDriveToAllignParamsNT.rotationKd.getValue());

        translationController.reset();
        rotationController.reset();
        swerve.setSwerveLimit(precisePointLimit);
    }

    @Override
    public void execute() {
        Pose2d poseWorldRobot = poseWorldRobotSupplier.get().toPose2d();
        finalDestination = finalDestinationSupplier.get();
        Rotation2d shiftingDirection = shiftingDirectionSupplier.get();
        driveTarget = getDriveTarget(poseWorldRobot, finalDestination, shiftingDirection);
        velocityWorldRobot = velocityWorldRobotSupplier.get();

        Pose2d poseRobotTarget = driveTarget.relativeTo(poseWorldRobot);
        Translation2d pRT = poseRobotTarget.getTranslation();
        double pRTNorm = pRT.getNorm();
        Rotation2d pRTDir = toAngle(pRT);

        // Positive command magnitude drives toward pRTDir.
        double vRTNorm = -translationController.calculate(pRTNorm, 0.0);
        Translation2d vRT = new Translation2d(vRTNorm, pRTDir);
        double omegaRT =
                -rotationController.calculate(poseRobotTarget.getRotation().getRadians(), 0.0);

        ChassisSpeeds desired = new ChassisSpeeds(vRT.getX(), vRT.getY(), omegaRT);
        ChassisSpeeds limited =
                precisePointLimit.apply(swerve.getChassisSpeeds(), desired, kDefaultDtSeconds);
        swerve.runTwist(limited);

        Logger.recordOutput(kTag + "/poseWorldTarget", driveTarget);
        Logger.recordOutput(kTag + "/finalDestination", finalDestination);
    }

    private Pose2d getDriveTarget(Pose2d robot, Pose2d goal, Rotation2d shiftingDirection) {
        Transform2d offset =
                new Transform2d(
                        new Pose2d(goal.getTranslation(), shiftingDirection),
                        new Pose2d(robot.getTranslation(), shiftingDirection));
        double yDistance = Math.abs(offset.getY());
        double xDistance = Math.abs(offset.getX());

        double parallelScale =
                MathUtil.clamp(
                        (yDistance / (Math.max(lateralShiftMaxMeters, kShiftEps) * 2.0))
                                + ((xDistance - 0.3)
                                        / (Math.max(parallelShiftMaxMeters, kShiftEps) * 3.0)),
                        0.0,
                        1.0);
        double lateralScale =
                MathUtil.clamp(
                        yDistance <= 0.2
                                ? 0.0
                                : xDistance / Math.max(parallelShiftMaxMeters, kShiftEps),
                        0.0,
                        1.0);

        if (parallelScale < SwerveDriveToAllignParamsNT.shiftingTerminate.getValue()) {
            parallelScale = 0.0;
        }
        if (lateralScale < SwerveDriveToAllignParamsNT.shiftingTerminate.getValue()) {
            lateralScale = 0.0;
        }

        double parallelShift = parallelScale * parallelShiftMaxMeters;
        double lateralShift = Math.copySign(lateralScale * lateralShiftMaxMeters, offset.getY());
        Translation2d shiftWorld =
                new Translation2d(parallelShift, lateralShift).rotateBy(shiftingDirection);
        return new Pose2d(goal.getTranslation().plus(shiftWorld), goal.getRotation());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setSwerveLimitDefault();
        swerve.runStop();
    }

    @Override
    public boolean isFinished() {
        Pose2d poseWorldRobot = poseWorldRobotSupplier.get().toPose2d();
        Pose2d poseRobotFinal = finalDestination.relativeTo(poseWorldRobot);
        Pose2d velocity = velocityWorldRobotSupplier.get();

        boolean translationOnTarget =
                epsilonEquals(
                        poseRobotFinal.getTranslation().getNorm(),
                        0.0,
                        SwerveDriveToAllignParamsNT.translationToleranceM.getValue());
        boolean rotationOnTarget =
                epsilonEquals(
                        poseRobotFinal.getRotation().getDegrees(),
                        0.0,
                        SwerveDriveToAllignParamsNT.rotationToleranceDeg.getValue());

        boolean translationStationary =
                epsilonEquals(
                                velocity.getTranslation().getX(),
                                0.0,
                                SwerveDriveToAllignParamsNT.translationStationaryMps.getValue())
                        && epsilonEquals(
                                velocity.getTranslation().getY(),
                                0.0,
                                SwerveDriveToAllignParamsNT.translationStationaryMps.getValue());
        boolean rotationStationary =
                epsilonEquals(
                        velocity.getRotation().getDegrees(),
                        0.0,
                        SwerveDriveToAllignParamsNT.rotationStationaryDegps.getValue());

        Logger.recordOutput(kTag + "/translationOnTarget", translationOnTarget);
        Logger.recordOutput(kTag + "/rotationOnTarget", rotationOnTarget);
        Logger.recordOutput(kTag + "/translationStationary", translationStationary);
        Logger.recordOutput(kTag + "/rotationStationary", rotationStationary);

        return translationOnTarget
                && rotationOnTarget
                && translationStationary
                && rotationStationary;
    }

    @NTParameter(tableName = "Params/" + kTag)
    public static class SwerveDriveToAllignParams {
        static final double translationKp = 6;
        static final double translationKi = 0.0;
        static final double translationKiZone = 0.0;
        static final double translationKd = 0.1;
        static final double translationToleranceM = 0.05;

        static final double rotationKp = 5.0;
        static final double rotationKi = 0.0;
        static final double rotationKiZone = 0.0;
        static final double rotationKd = 0.1;
        static final double rotationToleranceDeg = 2.0;

        static final double translationStationaryMps = 0.20;
        static final double rotationStationaryDegps = 10.0;
        static final double shiftingTerminate = 0;
    }
}
