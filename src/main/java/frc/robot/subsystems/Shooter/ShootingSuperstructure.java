package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.commands.AutoAimCommand;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import lib.ironpulse.subsystem.velocity.VelocityMotorSubsystem;
import lib.ironpulse.swerve.Swerve;
import org.littletonrobotics.junction.Logger;

/**
 * Coordinates a drum-shooter shot: flywheel speed + hood angle + feed gating, driven by distance to
 * the hub via {@link ShotCalculator}.
 *
 * <p>Unlike a turret robot, yaw is actuated by the chassis (see {@link AutoAimCommand}). This
 * superstructure deliberately does NOT own swerve — it only reads swerve pose to compute distance
 * and heading error. Compose {@link #aimAndShoot()} in parallel with an {@link AutoAimCommand} so the
 * drivetrain handles yaw while this handles hood + flywheel + feed.
 */
public class ShootingSuperstructure extends SubsystemBase {
    private final VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooter;
    private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood;
    private final HopperSubsystem hopper;
    private final Swerve swerve;
    private final ShotCalculator calculator = new ShotCalculator();

    public ShootingSuperstructure(
            VelocityMotorSubsystem<MotorInputsAutoLogged, MotorIO> shooter,
            PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood,
            HopperSubsystem hopper,
            Swerve swerve) {
        this.shooter = shooter;
        this.hood = hood;
        this.hopper = hopper;
        this.swerve = swerve;
        cachedAimHeading = AutoAimCommand.getShooterAimHeading(robotPose());
    }

    private Pose2d robotPose() {
        return swerve.getEstimatedPose().toPose2d();
    }

    /** Horizontal distance from the robot to the (alliance-flipped) hub, in meters. */
    public double distanceToTarget() {
        return AutoAimCommand.getDistanceToTarget(robotPose().getTranslation());
    }

    /**
     * Distance the shot must actually cover given chassis motion (shoot-on-move lookahead): the ball
     * inherits the chassis field velocity for its time of flight. Equals {@link #distanceToTarget()}
     * when stationary. Uses the commanded (setpoint) velocity for smoothness, à la 6328.
     */
    public double effectiveDistanceToTarget() {
        ChassisSpeeds fieldVel =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        swerve.getChassisSpeedsCmd(), robotPose().getRotation());
        return calculator.effectiveDistance(
                robotPose().getTranslation(),
                AutoAimCommand.getTarget(),
                fieldVel.vxMetersPerSecond,
                fieldVel.vyMetersPerSecond);
    }

    /** The shot solution (hood angle + flywheel speed), shoot-on-move compensated. */
    public ShotSolution currentSolution() {
        return calculator.solve(effectiveDistanceToTarget());
    }

    private Rotation2d computeAimHeading() {
        ChassisSpeeds fv = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getChassisSpeedsCmd(), robotPose().getRotation());
        double tof = calculator.timeOfFlightFor(distanceToTarget());
        Translation2d lookahead = robotPose().getTranslation().plus(new Translation2d(fv.vxMetersPerSecond * tof, fv.vyMetersPerSecond * tof));
        return AutoAimCommand.getShooterAimHeading(new Pose2d(lookahead, robotPose().getRotation()));
    }


    private Rotation2d cachedAimHeading;
    private Rotation2d lastAimHeading;
    private double aimRate;
    private LinearFilter aimRateFilter = LinearFilter.movingAverage((int)(0.1 / RobotConstants.LOOPER_DT));

    public Rotation2d aimHeading() {
        return cachedAimHeading;
    }

    public double aimHeadingRateRadPerSec() {
        return aimRate;
    }

    private Angle clampHoodAngle(Angle angle) {
        return Degrees.of(
                MathUtil.clamp(
                        angle.in(Degrees),
                        HoodConfig.HOOD_MIN_ANGLE.in(Degrees),
                        HoodConfig.HOOD_MAX_ANGLE.in(Degrees)));
    }

    /** True when the SHOOTER is pointed at the hub within the (NT-tunable) heading tolerance. */
    public boolean headingAtGoal() {
        Pose2d pose = robotPose();
        // Uses the shooter-corrected aim heading (off-axis firing + edge-mount offset), not robot-forward.
        Rotation2d aimHeading = AutoAimCommand.getShooterAimHeading(pose);
        double errorDeg = Math.abs(pose.getRotation().minus(aimHeading).getDegrees());
        return errorDeg <= ShootingParamsNT.headingToleranceDeg.getValue();
    }

    /** All three shot DOFs satisfied: chassis aimed, hood at angle, flywheel up to speed. */
    public boolean readyToShoot() {
        return headingAtGoal() && hood.positionAtGoal() && shooter.velocityAtGoal();
    }

    /**
     * Spin the flywheel to the solution speed and drive the hood to the solution angle — both
     * tracking distance continuously — then feed once {@link #readyToShoot()}.
     *
     * <p>Requires shooter/hood/floor-roller, NOT swerve; run it in parallel with an
     * {@link AutoAimCommand} which owns chassis yaw.
     */
    public Command aimAndShoot() {
        return Commands.parallel(
                shooter.runVelVolt(() -> currentSolution().shooterSpeed()),
                hood.runMotionMagic(() -> clampHoodAngle(currentSolution().hoodAngle())),
                Commands.waitUntil(this::readyToShoot).andThen(hopper.feed()));
    }

    /**
     * Park the shot mechanisms: stop the flywheel and flatten the hood. The floor roller is left to
     * its own default command. Bind to {@code onFalse} of the aim trigger.
     */
    public Command idle() {
        return Commands.parallel(
                shooter.runVelVolt(() -> RotationsPerSecond.of(ShooterParamsNT.idleRPS.getValue())),
                hood.runMotionMagic(HoodConfig.HOOD_STOW_ANGLE));
    }

    /** Zero only the hood mechanism. */
    public Command zeroHood() {
        return hood.zeroCommand();
    }

    @Override
    public void periodic() {
        double geometric = distanceToTarget();
        double effective = effectiveDistanceToTarget();
        ShotSolution solution = calculator.solve(effective);
        Rotation2d heading = computeAimHeading();
        double raw = (lastAimHeading == null)
                        ? 0.0
                        : heading.minus(lastAimHeading).getRadians() / RobotConstants.LOOPER_DT;
        aimRate = aimRateFilter.calculate(raw);
        lastAimHeading = heading;
        cachedAimHeading = heading;






        Logger.recordOutput("Shooting/distanceMeters", geometric);
        Logger.recordOutput("Shooting/effectiveDistanceMeters", effective);
        Logger.recordOutput("Shooting/lookaheadDeltaMeters", effective - geometric);
        Logger.recordOutput("Shooting/hoodTargetDeg", solution.hoodAngle().in(Degrees));
        Logger.recordOutput("Shooting/shooterTargetRPS", solution.shooterSpeed().in(RotationsPerSecond));
        Logger.recordOutput("Shooting/headingAtGoal", headingAtGoal());
        Logger.recordOutput("Shooting/readyToShoot", readyToShoot());
    }
}
