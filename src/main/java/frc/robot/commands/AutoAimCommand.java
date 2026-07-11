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

    // Clamp shoot-on-move feedforward so a bad pose/rate sample cannot yank the chassis. Heading
    // damping uses the direct gyro rate rather than a loop-time-dependent numerical derivative.
    private static final double FF_CLAMP_RAD_PER_SEC = 2.0;

    // O-lock hysteresis: once settled and X-locked, only unlock when the commanded speed/omega
    // exceed the lock thresholds by this factor, so the wheels don't flip X<->drive at the edge.
    private static final double LOCK_EXIT_FACTOR = 2.0;

    // True while the chassis is X-locked in the settled state (see execute()). Reset on each start.
    private boolean oLocked = false;

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

    @Override
    public void initialize() {
        oLocked = false;
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

        // ---- 1. Alliance-aware target and shoot-on-move state ----
        Rotation2d target = targetHeading.get();
        double error = target.minus(robotPose.getRotation()).getRadians();
        double ffVel =
                MathUtil.clamp(
                        targetHeadingRate.getAsDouble(),
                        -FF_CLAMP_RAD_PER_SEC,
                        FF_CLAMP_RAD_PER_SEC);
        double measuredOmega = swerve.getYawVelocityRadPerSec();

        // Direct gyro-rate damping is independent of PIDController's assumed 20 ms derivative
        // period, so occasional scheduler overruns do not amplify the derivative term.
        double omega =
                ffVel
                        + AutoAimParamsNT.kP.getValue() * error
                        + AutoAimParamsNT.kD.getValue() * (ffVel - measuredOmega);
        double maxVel = AutoAimParamsNT.maxAngularVelRadPerSec.getValue();
        omega = MathUtil.clamp(omega, -maxVel, maxVel);

        // ---- 2. Driver translation (capped, driver-relative) ----
        double maxSpeed = AutoAimParamsNT.maxSpeedMPS.getValue();
        double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.1);
        double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), 0.1);
        double vNorm = Math.hypot(x, y) * maxSpeed;
        Translation2d v = new Translation2d(vNorm, new Rotation2d(x, y));
        // Driver-relative so "forward" isn't mirrored on red while aiming. Aim omega is unaffected.
        Rotation2d driverHeading =
                RobotStateRecorder.getPoseDriverRobotCurrent().getRotation().toRotation2d();

        // ---- 3. O-lock settle (6328 stopWithO) ----
        // When the driver isn't translating AND the aim is settled (both commanded speeds tiny),
        // X-lock the wheels to rigidly hold heading instead of dithering sub-stiction omegas the
        // drivetrain can't execute — that dither is the terminal wiggle. Hysteresis on exit keeps
        // the wheels from flipping X<->drive at the boundary. It never engages while translating
        // (shoot-on-move) — then it just drives.
        double lockLin = AutoAimParamsNT.lockLinearMetersPerSec.getValue();
        double lockOmega = AutoAimParamsNT.lockOmegaRadPerSec.getValue();
        if (oLocked) {
            if (vNorm > lockLin * LOCK_EXIT_FACTOR
                    || Math.abs(omega) > lockOmega * LOCK_EXIT_FACTOR) {
                oLocked = false;
            }
        } else if (vNorm < lockLin && Math.abs(omega) < lockOmega) {
            oLocked = true;
        }

        if (oLocked) {
            swerve.runStopAndLock();
        } else {
            swerve.runTwist(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            v.getX(), v.getY(), omega, driverHeading));
        }

        // ---- 4. Logging ----
        Logger.recordOutput("AutoAim/targetHeadingDeg", target.getDegrees());
        Logger.recordOutput("AutoAim/errorDeg", Math.toDegrees(error));
        Logger.recordOutput(
                "AutoAim/onTarget",
                Math.abs(error) <= Math.toRadians(AutoAimParamsNT.toleranceDeg.getValue()));
        Logger.recordOutput("AutoAim/oLocked", oLocked);
        Logger.recordOutput("AutoAim/omegaCmd", omega);
        Logger.recordOutput("AutoAim/omegaMeasured", measuredOmega);
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
     * Fixed heading-control gains. Simulation gives only a ballpark; retune on the real robot and
     * commit the resulting values here.
     */
    @NTParameter(tableName = "Params/AutoAim")
    public static final class AutoAimParams {
        // omega = targetRate + kP * headingError + kD * (targetRate - measuredYawRate)
        public static final double kP = 2.5;
        public static final double kD = 0.2;
        // Hard cap on commanded yaw rate (rad/s). Drivetrain caps are ~7.85 rad/s; keep some
        // margin.
        public static final double maxAngularVelRadPerSec = 6.0;
        public static final double toleranceDeg = 3.0;
        // Driver translation speed cap (m/s) while auto-aiming — slow, controlled aiming instead of
        // the full drivetrain limit.
        public static final double maxSpeedMPS = 1.0;
        // O-lock settle thresholds (6328 stopWithO): when both the commanded translation speed
        // (m/s) and yaw rate (rad/s) are below these, the wheels X-lock to hold heading instead of
        // dithering. lockOmega sets the effective heading deadband — raise if it still wiggles when
        // settled; lower if it won't hold. Exit is these × LOCK_EXIT_FACTOR (hysteresis).
        public static final double lockLinearMetersPerSec = 0.05;
        public static final double lockOmegaRadPerSec = 0.1;
    }
}
