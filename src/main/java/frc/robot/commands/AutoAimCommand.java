package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.RobotStateRecorder;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.utils.AllianceFlipUtil;
import lib.ntext.NTParameter;
import org.littletonrobotics.junction.Logger;

/**
 * Rotates the swerve to face the Hub while still letting the driver translate. Use this in
 * Commands.parallel() alongside hoodSubsystem.runMotionMagic(angleSupplier) to also adjust the hood
 * angle simultaneously.
 */
public class AutoAimCommand extends Command {
    // Heading-control gains are NT-tunable (Params/AutoAim) — see AutoAimParams at the bottom.

    private final Swerve swerve;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final Supplier<Rotation2d> targetHeading;
    private final DoubleSupplier targetHeadingRate;

    public enum TargetMode {
        AUTO,
        HUB,
        PASS_LEFT,
        PASS_RIGHT
    }

    private static TargetMode targetMode = TargetMode.AUTO;

    public static void setTargetMode(TargetMode mode) {
        targetMode = mode;
        Logger.recordOutput("AutoAim/TargetMode", targetMode.name());
    }

    public static TargetMode getTargetMode() {
        return targetMode;
    }

    // Heading control is a plain P + velocity-feedforward + velocity-damping law (à la 6328's
    // joystickDriveWhileLaunching), NOT a motion profile:
    //   omega = ffVel + kP*headingError + kD*(ffVel - measuredYawRate)
    // The kD term is derivative-on-MEASUREMENT (against the direct gyro rate), so it damps cleanly
    // without the differentiated-heading noise, and there's no trapezoid profile to march ahead of
    // the chassis and overshoot — the two things that made the old profiled+latched version ring.

    public AutoAimCommand(
            Swerve swerve,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Rotation2d> targetHeading,
            DoubleSupplier targetHeadingRate) {
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

    /**
     * The point the shooter should aim at, given where the robot is. Normally the hub, but once the
     * robot crosses into the neutral zone we switch to passing: aim at a point near our own driver
     * station so the ball is lobbed back to the alliance zone instead of contested at the hub.
     *
     * <p>Because the whole shot solution (hood angle + flywheel speed via {@link
     * frc.robot.subsystems.Shooter.ShootingSuperstructure}) is derived from this target, switching
     * it here makes the pass a real pass — the superstructure ranges to the pass point, not the
     * hub.
     */
    public static Translation2d getTarget(Translation2d robotPos) {
        return switch (targetMode) {
            case AUTO -> inNeutralZone(robotPos) ? getPassTarget(robotPos) : getHubTarget();
            case HUB -> getHubTarget();
            case PASS_LEFT -> getPassTargetLeft();
            case PASS_RIGHT -> getPassTargetRight();
        };
    }

    /** Robot-agnostic overload — resolves the robot position from the state recorder. */
    public static Translation2d getTarget() {
        return getTarget(
                RobotStateRecorder.getPoseWorldRobotCurrent().getTranslation().toTranslation2d());
    }

    private static Translation2d getHubTarget() {
        // Hub goal via the transform tree (stored blue, returned alliance-flipped). Equivalent to
        // AllianceFlipUtil.apply(FieldConstants.Hub.getTarget2d()) but sourced from
        // RobotStateRecorder.
        return RobotStateRecorder.getPoseWorldTargetCurrent(RobotStateRecorder.kFrameGoal)
                .getTranslation()
                .toTranslation2d();
    }

    /**
     * True when the robot is between the two field neutral-zone lines (world X, alliance-agnostic).
     */
    public static boolean inNeutralZone(Translation2d robotPos) {
        double x = robotPos.getX();
        return x >= FieldConstants.LinesVertical.neutralZoneNear
                && x <= FieldConstants.LinesVertical.neutralZoneFar;
    }

    /**
     * The pass point on the robot's own Y-half. Both mirrored points are alliance-flipped into the
     * world frame first, then we pick the one nearer the robot in Y — so on either alliance the
     * ball goes back down the robot's sideline and the aim never sweeps through the central hub.
     */
    public static Translation2d getPassTarget(Translation2d robotPos) {
        Translation2d left = getPassTargetLeft();
        Translation2d right = getPassTargetRight();
        return Math.abs(left.getY() - robotPos.getY()) <= Math.abs(right.getY() - robotPos.getY())
                ? left
                : right;
    }

    public static Translation2d getPassTargetLeft() {
        return AllianceFlipUtil.apply(FieldConstants.PassTargets.BLUE_LEFT);
    }

    public static Translation2d getPassTargetRight() {
        return AllianceFlipUtil.apply(FieldConstants.PassTargets.BLUE_RIGHT);
    }

    private static boolean isPassingTarget(Translation2d robotPos) {
        return switch (targetMode) {
            case AUTO -> inNeutralZone(robotPos);
            case HUB -> false;
            case PASS_LEFT, PASS_RIGHT -> true;
        };
    }

    public static double getDistanceToTarget(Translation2d robotPos) {
        return getTarget(robotPos).getDistance(robotPos);
    }

    /**
     * Chassis heading that points the SHOOTER (not robot-forward/intake) at the hub, à la 6328:
     * bearingToHub + asin(shooterLateralY / distance) + shooterFiringYaw.
     */
    public static Rotation2d getShooterAimHeading(Pose2d robotPose) {
        Translation2d target = getTarget(robotPose.getTranslation());
        Rotation2d bearing = target.minus(robotPose.getTranslation()).getAngle();
        double distance = target.getDistance(robotPose.getTranslation());
        Rotation2d lateralCorrection =
                new Rotation2d(
                        MathUtil.clamp(
                                Math.asin(
                                        MathUtil.clamp(
                                                ROBOT_TO_SHOOTER.getY() / distance, -1.0, 1.0)),
                                -Math.PI,
                                Math.PI));
        return bearing.plus(lateralCorrection).plus(ROBOT_TO_SHOOTER.getRotation());
    }

    @Override
    public void execute() {
        var robotPose = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
        boolean passing = isPassingTarget(robotPose.getTranslation());
        Translation2d toTarget =
                getTarget(robotPose.getTranslation())
                        .minus(robotPose.getTranslation()); // aiming vector

        Rotation2d target = targetHeading.get();
        double error = target.minus(robotPose.getRotation()).getRadians();
        double ffVel = targetHeadingRate.getAsDouble();
        double measureOmega = swerve.getYawVelocityRadPerSec();

        double kP = AutoAimParamsNT.kP.getValue();
        double kD = AutoAimParamsNT.kD.getValue();
        double maxVel = AutoAimParamsNT.maxAngularVelRadPerSec.getValue();

        // 6328-style heading law (joystickDriveWhileLaunching): feedforward the aim point's angular
        // velocity (shoot-on-move), P on the live heading error, and D on the VELOCITY error
        // against
        // the measured yaw rate. That last term is the damping: with a stationary target ffVel≈0 so
        // it is −kD·yawRate, a clean brake off the direct gyro signal. No profile, no settle latch
        // —
        // when aimed and still, all three terms are ~0, so omega falls to 0 on its own.
        double omega = ffVel + kP * error + kD * (ffVel - measureOmega);
        omega = MathUtil.clamp(omega, -maxVel, maxVel);

        boolean onTarget =
                Math.abs(error) < Math.toRadians(AutoAimParamsNT.toleranceDeg.getValue());

        // Joystick translation — same convention as driveWithJoystick
        double maxSpeed = AutoAimParamsNT.maxSpeedMPS.getValue();
        double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.1);
        double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), 0.1);
        double vNorm = Math.hypot(x, y) * maxSpeed;
        Translation2d v = new Translation2d(vNorm, new Rotation2d(x, y));
        // Driver-relative heading (same convention as driveWithJoystick) so translation isn't
        // mirrored on the red alliance while auto-aiming. The aim omega above is unaffected.
        Rotation2d driverHeading =
                RobotStateRecorder.getPoseDriverRobotCurrent().getRotation().toRotation2d();
        ChassisSpeeds speeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(v.getX(), v.getY(), omega, driverHeading);
        swerve.runTwist(speeds);

        Logger.recordOutput("AutoAim/TargetHeading", target.getDegrees());
        Logger.recordOutput("AutoAim/Distance", toTarget.getNorm());
        // Passing = aiming at the sideline pass point instead of the hub (robot in neutral zone).
        Logger.recordOutput("AutoAim/Passing", passing);
        Logger.recordOutput("AutoAim/TargetMode", targetMode.name());
        // Tuning observability — plot these over time to spot oscillation / steady-state error /
        // sign.
        Logger.recordOutput("AutoAim/errorDeg", Math.toDegrees(error));
        Logger.recordOutput("AutoAim/onTarget", onTarget);
        Logger.recordOutput("AutoAim/omegaCmd", omega);
        Logger.recordOutput("AutoAim/omegaMeas", measureOmega);
        Logger.recordOutput("AutoAim/ffVel", ffVel);
        // Reversal diagnosis — compare against the same quantities in teleop driveWithJoystick
        // while
        // pushing the stick straight forward. stickX should be +forward, stickY +left.
        // driverHeadingDeg is the robot heading in the driver-station frame fed to
        // fromFieldRelativeSpeeds; if it reads ~0 while the robot is clearly rotated, the transform
        // lookup is falling back to identity. speedsVx/Vy are the resulting robot-frame commands.
        Logger.recordOutput("AutoAim/stickX", x);
        Logger.recordOutput("AutoAim/stickY", y);
        Logger.recordOutput("AutoAim/driverHeadingDeg", driverHeading.getDegrees());
        Logger.recordOutput("AutoAim/speedsVx", speeds.vxMetersPerSecond);
        Logger.recordOutput("AutoAim/speedsVy", speeds.vyMetersPerSecond);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.runStop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * NT-tunable heading-control gains for live tuning (sim or real) without recompiling. Generates
     * AutoAimParamsNT; adjust under Params/AutoAim in AdvantageScope while holding aim.
     *
     * <p>TODO: sim gives only a ballpark (idealized SimpleSim rotational dynamics) — retune kP/kD
     * on the real robot.
     */
    @NTParameter(tableName = "Params/AutoAim")
    public static final class AutoAimParams {
        // omega = ffVel + kP*headingError + kD*(ffVel - measuredYawRate). Defaults are 6328's
        // launching-heading gains (their DriveCommands/Launching kP/kD). kP drives to the target;
        // kD
        // damps off the measured yaw rate (derivative-on-measurement) and does NOT fast-flip
        // because
        // it isn't a differentiated heading. Retune on the real robot if needed.
        public static final double kP = 8.0;
        public static final double kD = 0.5;
        // Hard cap on commanded yaw rate (rad/s). Drivetrain caps are ~7.85 rad/s; keep some
        // margin.
        public static final double maxAngularVelRadPerSec = 6.0;
        // Heading error under which we report onTarget (observability / external "aimed" gate).
        // Loose
        // is fine — final precision comes from the flywheel/hood solution and shoot-on-move, à la
        // 6328's 10° launching tolerance.
        public static final double toleranceDeg = 3.0;
        // Driver translation speed cap (m/s) while auto-aiming — slow, controlled aiming instead of
        // the full drivetrain limit. Kept from the "pid" commit.
        public static final double maxSpeedMPS = 1.0;
    }
}
