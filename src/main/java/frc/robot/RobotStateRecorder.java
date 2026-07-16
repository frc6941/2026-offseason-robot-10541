package frc.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static lib.ironpulse.math.MathTools.toPose2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import lib.ironpulse.math.rbd.TransformRecorder;
import lib.ironpulse.utils.AllianceFlipUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * A trimmed {@link TransformRecorder} (transform-tree) for the fixed-shooter offseason robot.
 *
 * <p>Templated on the competition robot's RobotStateRecorder but stripped of the turret/hood shot
 * frame, ShotFrame, and obstacle zones — this robot has none of those. What's left is the part that
 * earns its keep here: a single canonical place where robot pose, velocity, and alliance-flipped
 * targets live, so no call site re-reads the swerve or re-applies {@code shouldFlip()} by hand.
 *
 * <p>Frames (all target frames are stored in the blue reference and read back alliance-flipped via
 * {@link #getPoseWorldTargetCurrent}):
 *
 * <ul>
 *   <li>{@code World} — the blue-origin field frame. All poses are stored here.
 *   <li>{@code Robot} — dynamic, World->Robot, refreshed every loop from the swerve pose estimator.
 *   <li>{@code DriverStationBlue/Red} — static World->DS transforms (from {@link
 *       TransformRecorder}). Driving relative to the alliance DS frame is how the 180deg flip is
 *       applied without any hand-written {@code shouldFlip()} at the call site.
 *   <li>{@code Goal} — static World->Hub (blue reference); the shooter's aim target.
 *   <li>{@code PassLeft/PassRight} — static World->pass-target points on each Y-half; the aim
 *       target while lobbing back from the neutral zone.
 * </ul>
 *
 * <p>Beyond the frame tree it also caches the robot's chassis velocity — measured + commanded
 * (robot-relative), each in its own time-interpolated buffer, plus the IMU yaw rate — refreshed
 * every loop from the same {@code updateDashboard} feed as the World->Robot pose. Velocity is
 * stored as a {@link Pose2d} ({@code x=vx, y=vy, rotation=omega}) exactly like the competition
 * robot's recorder, so anything needing the robot's pose OR velocity has a single source and nobody
 * re-reads the swerve directly.
 */
public class RobotStateRecorder extends TransformRecorder {
    public static final String kFrameGoal = "Goal";
    public static final String kFramePassLeft = "PassLeft";
    public static final String kFramePassRight = "PassRight";

    private static RobotStateRecorder instance;

    // Robot chassis velocity, refreshed every loop from the swerve alongside the World->Robot pose
    // (see RobotContainer.updateDashboard). Buffered by timestamp — like the poses — so velocity
    // and
    // pose sample consistently and no consumer reads the swerve directly. Stored as Pose2d via
    // MathTools.toPose2d: x=vx, y=vy, rotation=omega.
    private static TimeInterpolatableBuffer<Pose2d> velocityRobotBuffer;
    private static TimeInterpolatableBuffer<Pose2d> velocityRobotCmdBuffer;
    private static AngularVelocity omegaRobotCurrent = RadiansPerSecond.zero();

    @AutoLogOutput(key = "RobotStateRecorder/kFrameTarget")
    private static String kFrameTarget = kFrameGoal;

    private RobotStateRecorder() {
        setBufferDuration(2.0);
        velocityRobotBuffer = TimeInterpolatableBuffer.createBuffer(2.0);
        velocityRobotCmdBuffer = TimeInterpolatableBuffer.createBuffer(2.0);
        // static: World->DriverStation (per alliance)
        putTransform(kTransformWorldDriverStationBlue, kFrameWorld, kFrameDriverStationBlue);
        putTransform(kTransformWorldDriverStationRed, kFrameWorld, kFrameDriverStationRed);
        // dynamic: World->Robot, seeded at origin until the first periodic update
        putTransform(new Pose3d(), Seconds.of(0.0), kFrameWorld, kFrameRobot);
        // static: World->Goal (blue reference; flipped on read in getPoseWorldTargetCurrent)
        putTransform(
                new Pose3d(FieldConstants.Hub.TARGET, Rotation3d.kZero),
                Seconds.of(0.0),
                kFrameWorld,
                kFrameGoal);
        // static: World->pass targets (blue reference; flipped on read). Same treatment as the Hub
        // so AutoAim's neutral-zone pass points come from the tree, not an inline AllianceFlipUtil.
        putTransform(
                new Pose3d(new Pose2d(FieldConstants.PassTargets.BLUE_LEFT, Rotation2d.kZero)),
                Seconds.of(0.0),
                kFrameWorld,
                kFramePassLeft);
        putTransform(
                new Pose3d(new Pose2d(FieldConstants.PassTargets.BLUE_RIGHT, Rotation2d.kZero)),
                Seconds.of(0.0),
                kFrameWorld,
                kFramePassRight);
    }

    public static RobotStateRecorder getInstance() {
        if (instance == null) {
            instance = new RobotStateRecorder();
        }
        return instance;
    }

    public static void periodic() {
        Logger.recordOutput("RobotStateRecorder/poseWorldRobot", getPoseWorldRobotCurrent());
        Logger.recordOutput(
                "RobotStateRecorder/RobotRotation2d",
                getPoseWorldRobotCurrent().getRotation().toRotation2d());
        Logger.recordOutput(
                "RobotStateRecorder/TargetPoseWorld", getPoseWorldTargetCurrent(kFrameTarget));
        Logger.recordOutput("RobotStateRecorder/velocityRobot", getVelocityRobotCurrent());
        Logger.recordOutput(
                "RobotStateRecorder/velocityWorldRobot", getVelocityWorldRobotCurrent());
        Logger.recordOutput("RobotStateRecorder/velocityRobotCmd", getVelocityRobotCmdCurrent());
        Logger.recordOutput(
                "RobotStateRecorder/velocityWorldRobotCmd", getVelocityWorldRobotCmdCurrent());
        Logger.recordOutput(
                "RobotStateRecorder/omegaRobotCurrentRadPerSec",
                getOmegaRobotCurrent().in(RadiansPerSecond));
    }

    // ---- State feed (called each loop from RobotContainer.updateDashboard) ----

    /**
     * Push one full robot-state sample — World->Robot pose plus measured/commanded velocity and yaw
     * rate — in a single call. The caller (the loop that owns the swerve) passes plain values, so
     * the recorder stays free of any {@code Swerve} dependency while all the buffering lives here
     * and pose + velocity land under one shared {@code time} key.
     */
    public static void putRobotState(
            Time time,
            Pose3d poseWorldRobot,
            ChassisSpeeds measured,
            ChassisSpeeds commanded,
            AngularVelocity yawRate) {
        getInstance().putTransform(poseWorldRobot, time, kFrameWorld, kFrameRobot);
        putVelocityRobot(time, measured);
        putVelocityRobotCmd(time, commanded);
        putOmegaRobotCurrent(yawRate);
    }

    /** Buffer the robot-relative measured chassis velocity ({@code swerve.getChassisSpeeds()}). */
    public static void putVelocityRobot(Time time, ChassisSpeeds speed) {
        velocityRobotBuffer.addSample(time.in(Seconds), toPose2d(speed));
    }

    /** Buffer the robot-relative commanded chassis velocity ({@code getChassisSpeedsCmd()}). */
    public static void putVelocityRobotCmd(Time time, ChassisSpeeds speedCmd) {
        velocityRobotCmdBuffer.addSample(time.in(Seconds), toPose2d(speedCmd));
    }

    /** Latch the IMU yaw rate ({@code swerve.getYawVelocityRadPerSec()}). */
    public static void putOmegaRobotCurrent(AngularVelocity omega) {
        omegaRobotCurrent = omega;
    }

    // ---- Velocity reads (Pose2d: x=vx, y=vy, rotation=omega) ----

    /** Robot-relative measured velocity as {@code (vx, vy, omega)}. */
    public static Pose2d getVelocityRobotCurrent() {
        return velocityRobotBuffer.getSample(Timer.getTimestamp()).orElse(new Pose2d());
    }

    /** Robot-relative commanded velocity as {@code (vx, vy, omega)}. */
    public static Pose2d getVelocityRobotCmdCurrent() {
        return velocityRobotCmdBuffer.getSample(Timer.getTimestamp()).orElse(new Pose2d());
    }

    /** IMU yaw rate. */
    public static AngularVelocity getOmegaRobotCurrent() {
        return omegaRobotCurrent;
    }

    /** Field-relative measured velocity (robot velocity rotated by the current heading). */
    public static Pose2d getVelocityWorldRobotCurrent() {
        return toWorldVelocity(getVelocityRobotCurrent());
    }

    /** Field-relative commanded velocity (robot velocity rotated by the current heading). */
    public static Pose2d getVelocityWorldRobotCmdCurrent() {
        return toWorldVelocity(getVelocityRobotCmdCurrent());
    }

    // Rotate a robot-relative velocity (vx, vy, omega) into the world frame by the robot heading,
    // preserving the angular component. Shared by the measured/commanded world-velocity getters.
    private static Pose2d toWorldVelocity(Pose2d velocityRobot) {
        Rotation2d heading = getPoseWorldRobotCurrent().toPose2d().getRotation();
        Translation2d velWorldTrans = velocityRobot.getTranslation().rotateBy(heading);
        return new Pose2d(velWorldTrans, velocityRobot.getRotation());
    }

    /** Robot-relative measured chassis speeds (matches {@code swerve.getChassisSpeeds()}). */
    public static ChassisSpeeds getChassisSpeeds() {
        Pose2d v = getVelocityRobotCurrent();
        return new ChassisSpeeds(v.getX(), v.getY(), v.getRotation().getRadians());
    }

    /** Robot pose in the blue-origin world frame (matches {@code swerve.getEstimatedPose()}). */
    public static Pose3d getPoseWorldRobotCurrent() {
        return getInstance()
                .getTransform(
                        Seconds.of(Timer.getTimestamp()),
                        TransformRecorder.kFrameWorld,
                        TransformRecorder.kFrameRobot)
                .orElse(new Pose3d());
    }

    /**
     * Robot pose expressed in the current alliance's DriverStation frame. Feed this to {@code
     * SwerveCommands.driveWithJoystick} as the pose supplier so "forward" on the stick means
     * away-from-driver on both alliances — no inline {@code shouldFlip()} needed.
     */
    public static Pose3d getPoseDriverRobotCurrent() {
        return getInstance()
                .getTransform(
                        Seconds.of(Timer.getTimestamp()),
                        DriverStation.getAlliance()
                                        .orElse(DriverStation.Alliance.Blue)
                                        .equals(DriverStation.Alliance.Blue)
                                ? TransformRecorder.kFrameDriverStationBlue
                                : TransformRecorder.kFrameDriverStationRed,
                        TransformRecorder.kFrameRobot)
                .orElse(new Pose3d());
    }

    /** World->target pose, alliance-flipped (target frames are stored in the blue reference). */
    public static Pose3d getPoseWorldTargetCurrent(String targetFrame) {
        Pose3d poseBlue =
                getInstance()
                        .getTransform(
                                Seconds.of(Timer.getTimestamp()),
                                TransformRecorder.kFrameWorld,
                                targetFrame)
                        .orElse(new Pose3d());
        return AllianceFlipUtil.apply(poseBlue);
    }

    /**
     * Alliance-flipped target translation (2d) for {@code targetFrame} — e.g. {@link #kFrameGoal},
     * {@link #kFramePassLeft}, {@link #kFramePassRight}. Lets aim/range code fetch a target without
     * an inline {@code AllianceFlipUtil.apply(FieldConstants...)}.
     */
    public static Translation2d getTranslationTargetCurrent(String targetFrame) {
        return getPoseWorldTargetCurrent(targetFrame).getTranslation().toTranslation2d();
    }
}
