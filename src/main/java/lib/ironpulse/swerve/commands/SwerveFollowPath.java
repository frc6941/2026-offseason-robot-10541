package lib.ironpulse.swerve.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static lib.ironpulse.math.MathTools.epsilonEquals;
import static lib.ironpulse.math.MathTools.toAngle;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import lib.ironpulse.swerve.Swerve;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

/**
 * Follows a {@link PathPlannerPath} using PathPlanner 2026's on-the-fly trajectory generation.
 *
 * <p>In {@link #initialize()}, the path is converted to a {@link PathPlannerTrajectory} via {@link
 * PathPlannerPath#generateTrajectory(ChassisSpeeds, Rotation2d, RobotConfig)}. The generated
 * trajectory is then followed with the same feedforward + PID feedback control loop used by {@link
 * SwerveFollowPathPlannerTrajectory}.
 *
 * <p>Event markers on the path are NOT handled by this command — use {@link
 * edu.wpi.first.wpilibj2.command.Commands#deadlineFor} or {@link
 * edu.wpi.first.wpilibj2.command.Commands#race} to compose path following with intake/shoot
 * commands. This matches Team 6328's approach in {@code AutoCommands.java}.
 *
 * <p>A {@link RobotConfig} must be set via {@link #setRobotConfig(RobotConfig)} before scheduling.
 *
 * <p>Pattern adapted from Team 6328's {@code DriveTrajectory.java} (Choreo-based) and the local
 * {@link SwerveFollowPathPlannerTrajectory}.
 */
public class SwerveFollowPath extends Command {
    private final Swerve swerve;
    private final PathPlannerPath path;

    // --- Settable configuration (required before scheduling) ---
    @Setter private Supplier<Pose3d> poseWorldRobotSupplier;
    @Setter private Distance translationTolerance = Meters.of(0.05);
    @Setter private Angle rotationTolerance = Radians.of(0.05);
    @Setter private PIDController translationController;
    @Setter private PIDController rotationController;
    @Setter private RobotConfig robotConfig;

    // --- Optional configuration ---
    @Setter private EndStrategy endStrategy = EndStrategy.EndWithTime;
    @Setter private Strategy strategy = Strategy.PurePursuit;

    // --- Runtime state ---
    private PathPlannerTrajectory generatedTrajectory;
    private final Timer trajectoryTimer = new Timer();

    public SwerveFollowPath(
            Swerve swerve,
            PathPlannerPath path,
            PIDController translationController,
            PIDController rotationController) {
        this.swerve = swerve;
        this.path = path;
        this.translationController = translationController;
        this.rotationController = rotationController;
        addRequirements(swerve);
    }

    // ========================================================================
    // Lifecycle — mirrors SwerveFollowPathPlannerTrajectory
    // ========================================================================

    @Override
    public void initialize() {
        translationController.reset();
        rotationController.reset();
        trajectoryTimer.reset();
        trajectoryTimer.start();

        // On-the-fly trajectory generation (PathPlanner 2026 API)
        // Starting from rest at the current robot heading
        try {
            Pose3d robotPose = poseWorldRobotSupplier.get();
            Rotation2d startHeading = robotPose.getRotation().toRotation2d();
            ChassisSpeeds startingSpeeds = new ChassisSpeeds();

            generatedTrajectory =
                    path.generateTrajectory(startingSpeeds, startHeading, robotConfig);

            if (generatedTrajectory == null
                    || generatedTrajectory.getTotalTimeSeconds() <= 1e-6) {
                DriverStation.reportError(
                        "[SwerveFollowPath] Trajectory generation produced an empty trajectory "
                                + "(totalTime="
                                + (generatedTrajectory == null
                                        ? "null"
                                        : generatedTrajectory.getTotalTimeSeconds())
                                + "). Path has "
                                + path.numPoints()
                                + " points.",
                        false);
            }

            Logger.recordOutput(
                    "SwerveFollowPath/TotalTime",
                    generatedTrajectory != null
                            ? generatedTrajectory.getTotalTimeSeconds()
                            : -1.0);
        } catch (Exception e) {
            DriverStation.reportError(
                    "[SwerveFollowPath] Trajectory generation failed: " + e.getMessage(),
                    e.getStackTrace());
            generatedTrajectory = null;
        }
    }

    @Override
    public void execute() {
        if (generatedTrajectory == null) {
            return;
        }

        double t = trajectoryTimer.get();
        Pose2d TWR = poseWorldRobotSupplier.get().toPose2d();
        PathPlannerTrajectoryState trajectoryState = generatedTrajectory.sample(t);

        // --- Feedforward ---
        ChassisSpeeds V_FF = trajectoryState.fieldSpeeds;
        Current[] tau_FF = trajectoryState.feedforwards.torqueCurrents();

        // --- Translation feedback ---
        Pose2d TWT = trajectoryState.pose;
        Pose2d TRT = TWT.relativeTo(TWR);
        Translation2d pRT = TRT.getTranslation();
        double pRT_norm = pRT.getNorm();
        Rotation2d pRT_dir = toAngle(pRT);
        // pRT_norm is always positive, so vRT_norm is always negative.
        // Take the minus sign so the robot moves along (not opposite to) pRT_dir.
        double vRT_norm = translationController.calculate(pRT_norm, 0.0);
        Translation2d vRT = new Translation2d(-vRT_norm, pRT_dir);

        // --- Rotation feedback ---
        double thetaRT = TRT.getRotation().getRadians();
        double omegaRT = rotationController.calculate(thetaRT, 0.0);

        ChassisSpeeds V_FB = new ChassisSpeeds(vRT.getX(), vRT.getY(), omegaRT);

        // --- Combined FF + FB command ---
        swerve.runTwistWithTorque(V_FF.plus(V_FB), tau_FF);
    }

    @Override
    public void end(boolean interrupted) {
        if (generatedTrajectory != null) {
            swerve.runTwist(generatedTrajectory.getEndState().fieldSpeeds);
        } else {
            swerve.runStop();
        }
    }

    @Override
    public boolean isFinished() {
        if (generatedTrajectory == null
                || generatedTrajectory.getTotalTimeSeconds() <= 1e-6) {
            return true;
        }
        boolean isTimeout =
                trajectoryTimer.hasElapsed(generatedTrajectory.getTotalTimeSeconds());

        if (endStrategy == EndStrategy.EndWithTimeAndPose) {
            Pose2d poseWorldRobotCurrent = poseWorldRobotSupplier.get().toPose2d();
            Pose2d poseWorldTrajectoryEnd = generatedTrajectory.getEndState().pose;
            boolean isOnTarget =
                    epsilonEquals(
                                    poseWorldRobotCurrent.getTranslation(),
                                    poseWorldTrajectoryEnd.getTranslation(),
                                    translationTolerance.in(Meters))
                            && epsilonEquals(
                                    poseWorldRobotCurrent.getRotation(),
                                    poseWorldTrajectoryEnd.getRotation(),
                                    rotationTolerance.in(Radians));
            return isTimeout && isOnTarget;
        }

        return isTimeout;
    }

    // ========================================================================
    // Enums
    // ========================================================================

    public enum EndStrategy {
        /** Finish when the trajectory time expires (default). */
        EndWithTime,
        /** Finish when time expires AND the robot is within tolerance of the final pose. */
        EndWithTimeAndPose
    }

    public enum Strategy {
        /** Pure pursuit: track the closest point + lookahead distance. */
        PurePursuit,
        /** Adaptive pure pursuit: dynamically adjust lookahead based on speed. */
        AdaptivePurePursuit,
        /** Regulated pure pursuit: use velocity regulation to stay on path. */
        RegulatedPurePursuit
    }
}
