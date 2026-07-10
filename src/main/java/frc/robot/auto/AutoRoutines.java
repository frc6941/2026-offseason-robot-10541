package frc.robot.auto;

import static frc.robot.auto.AutoActions.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.subsystems.Shooter.ShootingSuperstructure;
import java.util.Set;
import lib.ironpulse.swerve.Swerve;

/**
 * Composed autonomous routines, ported from the competition robot's {@code AutoRoutines}.
 *
 * <p>Every routine follows the same non-turret shape: <b>collect</b> balls while following a
 * PathPlanner path with the intake running, then <b>drive to a shoot pose and empty the hopper</b>
 * (the robot cannot shoot while moving like the turreted competition robot).
 *
 * <p>Path names (e.g. {@code "sweepRight"}) refer to {@code .path} files you author in the
 * PathPlanner GUI under {@code deploy/pathplanner/paths}. Mirror = true reuses one path for the
 * opposite side.
 */
public class AutoRoutines {
    public static Swerve swerve;
    public static ShootingSuperstructure shootingSuperstructure;
    public static IntakerSubsystem intake;

    public static void init(
            Swerve swerve, ShootingSuperstructure shootingSuperstructure, IntakerSubsystem intake) {
        AutoRoutines.swerve = swerve;
        AutoRoutines.shootingSuperstructure = shootingSuperstructure;
        AutoRoutines.intake = intake;
    }

    /** Wrap any routine so the mechanisms home while it runs. */
    public static Command withZeroing(Command routine) {
        return Commands.parallel(routine, zeroEverything());
    }

    /** Sit still and empty the preload from the current pose (assumes we already face the hub). */
    public static Command shootPreload() {
        return Commands.deadline(shootAllBalls(), holdStill());
    }

    /**
     * One "collect then score" cycle: run {@code sweepPath} with the intake on, then drive to the
     * shoot pose and empty the hopper.
     */
    public static Command sweepAndShoot(String sweepPath, boolean isLeft) {
        return Commands.defer(
                () ->
                        Commands.sequence(
                                Commands.deadline(followPathFile(sweepPath, isLeft), intake()),
                                driveToShootAndFire(isLeft)),
                Set.of(swerve, shootingSuperstructure, intake));
    }

    /** Shoot the preload first, then run one sweep-and-shoot cycle. */
    public static Command preloadThenSweep(String sweepPath, boolean isLeft) {
        return Commands.defer(
                () ->
                        Commands.sequence(
                                driveToShootAndFire(isLeft),
                                Commands.deadline(followPathFile(sweepPath, isLeft), intake()),
                                driveToShootAndFire(isLeft)),
                Set.of(swerve, shootingSuperstructure, intake));
    }

    /** Two collect/score cycles, second cycle reuses the same sweep path. */
    public static Command doubleSweep(String sweepPath, boolean isLeft) {
        return Commands.defer(
                () ->
                        Commands.sequence(
                                sweepAndShoot(sweepPath, isLeft), sweepAndShoot(sweepPath, isLeft)),
                Set.of(swerve, shootingSuperstructure, intake));
    }

    /** Bare-bones diagnostic: drive the generated test path. */
    public static Command test() {
        return Commands.sequence(resetOnPose(AutoActions.kTestA), testPath());
    }
}
