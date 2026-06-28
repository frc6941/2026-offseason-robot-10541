package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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
 * frame, velocity buffers, ShotFrame, and obstacle zones — this robot has none of those. What's left
 * is the part that earns its keep here: a single canonical place where alliance handling lives.
 *
 * <p>Frames:
 *
 * <ul>
 *   <li>{@code World} — the blue-origin field frame. All poses are stored here.
 *   <li>{@code Robot} — dynamic, World->Robot, refreshed every loop from the swerve pose estimator.
 *   <li>{@code DriverStationBlue/Red} — static World->DS transforms (from {@link TransformRecorder}).
 *       Driving relative to the alliance DS frame is how the 180deg flip is applied without any
 *       hand-written {@code shouldFlip()} at the call site.
 *   <li>{@code Goal} — static World->Hub (blue reference); read back alliance-flipped via {@link
 *       #getPoseWorldTargetCurrent}.
 * </ul>
 */
public class RobotStateRecorder extends TransformRecorder {
    public static final String kFrameGoal = "Goal";

    private static RobotStateRecorder instance;

    @AutoLogOutput(key = "RobotStateRecorder/kFrameTarget")
    private static String kFrameTarget = kFrameGoal;

    private RobotStateRecorder() {
        setBufferDuration(2.0);
        // static: World->DriverStation (per alliance)
        putTransform(
                kTransformWorldDriverStationBlue, kFrameWorld, kFrameDriverStationBlue);
        putTransform(kTransformWorldDriverStationRed, kFrameWorld, kFrameDriverStationRed);
        // dynamic: World->Robot, seeded at origin until the first periodic update
        putTransform(new Pose3d(), Seconds.of(0.0), kFrameWorld, kFrameRobot);
        // static: World->Goal (blue reference; flipped on read in getPoseWorldTargetCurrent)
        putTransform(
                new Pose3d(FieldConstants.Hub.TARGET, Rotation3d.kZero),
                Seconds.of(0.0),
                kFrameWorld,
                kFrameGoal);
    }

    public static RobotStateRecorder getInstance() {
        if (instance == null) {
            instance = new RobotStateRecorder();
        }
        return instance;
    }

    public static void periodic() {
        Logger.recordOutput(
                "RobotStateRecorder/poseWorldRobot", getPoseWorldRobotCurrent());
        Logger.recordOutput(
                "RobotStateRecorder/RobotRotation2d",
                getPoseWorldRobotCurrent().getRotation().toRotation2d());
        Logger.recordOutput(
                "RobotStateRecorder/TargetPoseWorld", getPoseWorldTargetCurrent(kFrameTarget));
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
}
