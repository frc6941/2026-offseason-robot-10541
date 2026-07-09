package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

    // Heading is motion-profiled: the trapezoid plans deceleration from the remaining angle so the
    // chassis arrives at the target at zero velocity (overshoot-free), while still allowing an
    // aggressive max velocity/accel for a quick lock. Constraints/gains are refreshed live from NT.
    private final ProfiledPIDController headingController =
            new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));

    // Hysteresis latch for the terminal park handoff (see execute()). Enters within a tight aimed +
    // slow window, exits only past a wider error band, so the settle can't chatter at the edge.
    private boolean settled = false;

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
        // Heading wraps at ±π, so let the controller take the short way around.
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // Seed the profile with the robot's CURRENT heading and yaw rate so engaging aim mid-motion
        // (e.g. while already rotating/translating) doesn't cause a velocity discontinuity / jump.
        double headingRad = swerve.getEstimatedPose().toPose2d().getRotation().getRadians();
        headingController.reset(headingRad, swerve.getYawVelocityRadPerSec());
        settled = false;
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
        double headingRad = robotPose.getRotation().getRadians();
        double error = target.minus(robotPose.getRotation()).getRadians();
        double ffVel = targetHeadingRate.getAsDouble();
        double measureOmega = swerve.getYawVelocityRadPerSec();

        // Refresh gains + profile constraints live from NT. kP/kD now act on heading error against
        // the PROFILED setpoint (kD is a real derivative-on-error, no longer the old delayed-omega
        // feedback that rang). maxVel/maxAccel set how hard the profile drives the lock-on.
        double kP = AutoAimParamsNT.kP.getValue();
        double kD = AutoAimParamsNT.kD.getValue();
        double maxVel = AutoAimParamsNT.maxAngularVelRadPerSec.getValue();
        double maxAccel = AutoAimParamsNT.maxAngularAccelRadPerSec2.getValue();
        headingController.setP(kP);
        headingController.setD(kD);
        headingController.setConstraints(new TrapezoidProfile.Constraints(maxVel, maxAccel));
        headingController.setTolerance(Math.toRadians(AutoAimParamsNT.toleranceDeg.getValue()));

        // Goal carries the target's angular velocity (ffVel) so the profile tracks a moving aim
        // point
        // while you translate. Feedback drives heading→profiled setpoint; add the profile's own
        // velocity as feedforward so we move at profile speed even at zero position error.
        double feedback =
                headingController.calculate(
                        headingRad, new TrapezoidProfile.State(target.getRadians(), ffVel));
        double omega = feedback + headingController.getSetpoint().velocity;

        // Hysteresis park latch — kills the terminal limit cycle (slight overshoot + steady
        // wiggle).
        // ENTER "settled" only when aimed (|error| < toleranceDeg) AND slow (|yawRate| <
        // settleRateDegPerSec), so the kD braking above first arrests the chassis rather than
        // parking
        // mid-coast. Once settled, STAY settled — park on ffVel (~0 when stopped) — until the error
        // grows past the WIDER settleExitToleranceDeg band. The wider exit band is the whole point:
        // without it, a bare in-band check flips park↔PD every loop at the tolerance edge (that
        // chatter IS the wiggle) as drift/backlash/derivative-noise nudges the heading across the
        // boundary. With it, small disturbances are ignored and friction holds the aim; we only
        // re-engage the controller on a real change (target moved, got bumped).
        double toleranceRad = Math.toRadians(AutoAimParamsNT.toleranceDeg.getValue());
        double settleRateRad = Math.toRadians(AutoAimParamsNT.settleRateDegPerSec.getValue());
        double exitToleranceRad = Math.toRadians(AutoAimParamsNT.settleExitToleranceDeg.getValue());
        if (settled) {
            if (Math.abs(error) > exitToleranceRad) settled = false;
        } else if (Math.abs(error) < toleranceRad && Math.abs(measureOmega) < settleRateRad) {
            settled = true;
        }
        if (settled) {
            omega = ffVel;
        }
        omega = MathUtil.clamp(omega, -maxVel, maxVel);
        // When parked, zero tiny residual omega so the modules don't dither on an ill-defined
        // near-zero tangential azimuth target (the translational-jitter fix). Only when settled — a
        // genuine mid-approach correction must never be suppressed.
        if (settled && Math.abs(omega) < AutoAimParamsNT.outputDeadbandRadPerSec.getValue()) {
            omega = 0.0;
        }

        // Joystick translation — same convention as driveWithJoystick
        double maxSpeed = swerve.getSwerveLimit().maxLinearVelocity().in(MetersPerSecond);
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
        Logger.recordOutput("AutoAim/settled", settled);
        Logger.recordOutput("AutoAim/omegaCmd", omega);
        Logger.recordOutput("AutoAim/omegaMeas", measureOmega);
        Logger.recordOutput("AutoAim/ffVel", ffVel);
        // Profile tracking — setpointPosDeg should lead the heading and converge to target with no
        // overshoot; setpointVel is the planned (trapezoid) angular velocity.
        Logger.recordOutput(
                "AutoAim/setpointPosDeg", Math.toDegrees(headingController.getSetpoint().position));
        Logger.recordOutput("AutoAim/setpointVel", headingController.getSetpoint().velocity);
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
        // tracking error. Tuned on the real robot (2026-07-09): a stiff kP just saturated and rang;
        // the damping (kD) is what arrests the chassis at the target.
        public static final double kP = 3.0;
        public static final double kD = 0.6;
        // Profile limits — these set how fast the lock-on is. Drivetrain caps are 450°/s (7.85
        // rad/s)
        // and 5000°/s² (~87 rad/s²), but the real chassis can't track anywhere near that in yaw —
        // planning to unachievable accel made the heading lag the profile and overshoot. Tuned down
        // to what the robot actually follows (2026-07-09).
        public static final double maxAngularVelRadPerSec = 3.5;
        public static final double maxAngularAccelRadPerSec2 = 15.0;
        // Enter the parked "settled" state when heading error is within this band AND yaw rate is
        // below settleRateDegPerSec (see the hysteresis latch in execute()).
        public static final double toleranceDeg = 2.0;
        // Yaw-rate gate for entering "settled": the chassis must be aimed AND rotating slower than
        // this. Keeps kD braking active until the robot is actually slow, so it doesn't park
        // mid-coast
        // and overshoot. With the hysteresis exit below it's safe to keep this fairly loose.
        public static final double settleRateDegPerSec = 15.0;
        // Hysteresis EXIT band: once settled, stay parked until |error| exceeds this. Must be
        // comfortably larger than toleranceDeg (and larger than any residual overshoot amplitude)
        // or
        // the settle chatters back into PD and wiggles. Too large and it's slow to re-aim on a
        // bump.
        public static final double settleExitToleranceDeg = 4.0;
        // Chassis omega (rad/s) at or below this magnitude is floored to 0 (once withinTolerance)
        // before commanding the drivetrain. Kills the residual dither that scrubs the modules
        // (translational-feeling jitter) when settled on a stationary target. Raise if the lock
        // still buzzes; lower if the aim feels like it "sticks" and won't make fine corrections.
        public static final double outputDeadbandRadPerSec = 0.1;
    }
}
