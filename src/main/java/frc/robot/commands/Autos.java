package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import java.util.List;
import java.util.function.Supplier;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;

/**
 * Composed autonomous routines with shooting integration.
 *
 * <p>These routines use {@link SwerveCommands} + the completed {@code SwerveFollowPath}
 * infrastructure. Shooting and intake commands are accepted as {@link Supplier Suppliers} so each
 * step gets a fresh command instance — required by WPILib 2026's rule that composed commands cannot
 * be re-scheduled or re-composed.
 *
 * <p>Pattern adapted from Team 6328's {@code AutoBuilder.java}: declarative composition of {@link
 * Commands#sequence} and {@link Commands#parallel} (their {@code deadlineFor} equivalent).
 */
public final class Autos {
    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    // ========================================================================
    // Drive Forward (1 m/s for 2 seconds) — simplest possible auto
    // 6328 inspiration: driveForward1mSalesman
    // ========================================================================
    public static Command driveForward(Swerve swerve) {
        return Commands.sequence(
                SwerveCommands.reset(
                        swerve,
                        new Pose3d(FieldConstants.StartPositions.BLUE_CENTER)),
                Commands.run(
                                () -> swerve.runTwist(new ChassisSpeeds(1.0, 0.0, 0.0)),
                                swerve)
                        .withTimeout(2.0)
                        .andThen(Commands.runOnce(swerve::runStop, swerve)));
    }

    // ========================================================================
    // 1 Note — shoot the preload from center start
    // 6328 inspiration: timidSalesman
    // ========================================================================
    public static Command oneNote(Swerve swerve, Supplier<Command> shootSupplier) {
        return Commands.sequence(
                SwerveCommands.reset(
                        swerve,
                        new Pose3d(FieldConstants.StartPositions.BLUE_CENTER)),
                shootSupplier.get());
    }

    // ========================================================================
    // Example Path Auto — demonstrates SwerveFollowPath with a programmatic
    // S-curve path (no PathPlanner GUI required).  This is the reference
    // implementation showing how to construct a path in code and follow it
    // with feedforward+feedback control.
    //
    // Uses .beforeStarting() to reset the pose rather than Commands.sequence()
    // to avoid potential WPILib 2026 sequential-composition edge cases with
    // InstantCommand → custom Command transitions.
    // ========================================================================
    public static Command examplePathAuto(Swerve swerve, RobotConfig robotConfig) {
        PathPlannerPath path = buildExamplePath();
        PIDController transPID = new PIDController(5.0, 0.0, 0.0);
        PIDController rotPID = new PIDController(5.0, 0.0, 0.0);

        // Reset odometry before following the path.
        // .beforeStarting runs the Runnable in initialize() BEFORE generating
        // the trajectory, so the generator sees the correct starting pose.
        return SwerveCommands
                .followPath(
                        swerve,
                        path,
                        swerve::getEstimatedPose,
                        robotConfig,
                        transPID,
                        rotPID,
                        Meters.of(0.05),
                        Radians.of(0.05))
                .beforeStarting(
                        () -> swerve.resetEstimatedPose(
                                new Pose3d(path.getStartingDifferentialPose())));
    }

    // ========================================================================
    // Example Path + Shoot — combines programmatic path following with
    // shooting before and after the path.  Demonstrates the full 6328 pattern.
    // ========================================================================
    public static Command examplePathAndShoot(
            Swerve swerve,
            RobotConfig robotConfig,
            Supplier<Command> shootSupplier) {
        PathPlannerPath path = buildExamplePath();
        PIDController transPID = new PIDController(5.0, 0.0, 0.0);
        PIDController rotPID = new PIDController(5.0, 0.0, 0.0);

        // Using .beforeStarting() on the path command itself for the pose reset,
        // then composing with shoot commands via Commands.sequence().
        Command followPathWithReset = SwerveCommands
                .followPath(
                        swerve, path, swerve::getEstimatedPose, robotConfig,
                        transPID, rotPID,
                        Meters.of(0.05), Radians.of(0.05))
                .beforeStarting(
                        () -> swerve.resetEstimatedPose(
                                new Pose3d(path.getStartingDifferentialPose())));

        return Commands.sequence(
                // Shoot preload while stationary (already at correct pose from disabled)
                SwerveCommands.reset(
                        swerve,
                        new Pose3d(path.getStartingDifferentialPose())),
                shootSupplier.get(),
                // Follow the S-curve
                followPathWithReset,
                // Shoot again (fresh instance)
                Commands.waitSeconds(0.3),
                shootSupplier.get());
    }

    /**
     * Builds a 3-pose bezier S-curve path (blue-alliance coordinates).
     * Extracted so both example methods share the same path definition.
     */
    private static PathPlannerPath buildExamplePath() {
        List<Pose2d> poses = List.of(
                // Start at center, facing the field (0° = away from driver station)
                new Pose2d(0.76, FieldConstants.fieldWidth / 2.0, new Rotation2d()),
                // Drive forward to ~3.5m, starting a gentle right turn
                new Pose2d(3.5, 5.0, Rotation2d.fromDegrees(45.0)),
                // Finish at ~5.5m, turned 90° right, speed → 0
                new Pose2d(5.5, 6.0, Rotation2d.fromDegrees(90.0)));

        PathConstraints constraints = new PathConstraints(
                2.0, 2.5, // max velocity 2 m/s, max accel 2.5 m/s²
                Math.toRadians(360.0), Math.toRadians(540.0)); // angular limits

        GoalEndState goalEnd = new GoalEndState(0.0, Rotation2d.fromDegrees(90.0));

        return new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(poses),
                constraints,
                null, // idealStartingState (null = assume starting from rest)
                goalEnd);
    }

    // ========================================================================
    // 2 Note — shoot preload, sweep+intake, return, shoot again
    // 6328 inspiration: mellonomicsSalesman (simplified)
    //
    // Uses Commands.parallel to run intake while driving (6328's deadlineFor pattern).
    // Each call to shootSupplier/intakeSupplier creates a fresh command so WPILib's
    // "no re-scheduling composed commands" rule is satisfied.
    // ========================================================================
    public static Command twoNote(
            Swerve swerve,
            Supplier<Command> shootSupplier,
            Supplier<Command> intakeSupplier) {
        return Commands.sequence(
                // 1. Reset to center
                SwerveCommands.reset(
                        swerve,
                        new Pose3d(FieldConstants.StartPositions.BLUE_CENTER)),
                // 2. Shoot preload while stationary
                shootSupplier.get(),
                // 3. Drive forward 2s at 1.5 m/s while intaking
                Commands.parallel(
                        Commands.run(
                                        () -> swerve.runTwist(
                                                new ChassisSpeeds(1.5, 0.0, 0.0)),
                                        swerve)
                                .withTimeout(2.0)
                                .andThen(Commands.runOnce(swerve::runStop, swerve)),
                        intakeSupplier.get()),
                // 4. Settle
                Commands.waitSeconds(0.3),
                // 5. Shoot the acquired note (fresh command instance)
                shootSupplier.get());
    }
}
