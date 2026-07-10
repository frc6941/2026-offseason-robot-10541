package frc.robot.auto;

import static frc.robot.auto.AutoActions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.subsystems.Shooter.ShootingSuperstructure;
import java.util.ArrayList;
import java.util.List;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.utils.AllianceFlipUtil;

/**
 * Composed autonomous routines, ported from the competition robot's {@code AutoRoutines}.
 *
 * <p>The one competition routine has this shape (see {@link #competitionAuto}):
 *
 * <ol>
 *   <li>Follow the <b>start</b> path with the intake running (collect on the way out).
 *   <li>{@code drivePastSlope} back across the bump.
 *   <li>Aim the chassis at the hub and empty the hopper.
 *   <li>{@code driveToPose} to the hardcoded second-sweep start, run the <b>second sweep</b> path
 *       (intake on), come back, aim + shoot again — only when sweep count is 2.
 *   <li>Run the <b>end behaviour</b> (depot paths) with the intake running.
 * </ol>
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

    /** What to do after the sweep cycles. MIDDLE works both sides; DEPOT options are LEFT-only. */
    public enum EndBehaviour {
        NONE,
        MIDDLE,
        DEPOT,
        DEPOT_DRIVE_THROUGH
    }

    public static Command competitionAuto(
            boolean isLeft,
            boolean startFromBump,
            Pose2d blueStartPose,
            String startPath,
            Pose2d blueSecondSweepStart,
            String secondSweepPath,
            int sweepTimes,
            EndBehaviour endBehaviour) {
        List<Command> steps = new ArrayList<>();

        // 0. Sim-only pose reset so the robot starts where the path expects.
        steps.add(resetOnPose(blueStartPose));

        // 0b. Bump start begins on the bump — cross out into the neutral zone before anything else.
        if (startFromBump) {
            steps.add(drivePastSlope(isLeft, true));
        }

        // 1. First sweep: collect out, drive back, aim + shoot.
        steps.add(sweepCollectShoot(startPath, isLeft));

        // 2. Optional second sweep: reposition to the hardcoded start, then collect/shoot again.
        if (sweepTimes >= 2) {
            steps.add(driveToPose(AllianceFlipUtil.apply(blueSecondSweepStart)));
            steps.add(sweepCollectShoot(secondSweepPath, isLeft));
        }

        // 3. End behaviour (intake running throughout).
        steps.add(endBehaviour(endBehaviour, isLeft));

        return Commands.sequence(steps.toArray(Command[]::new))
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    /**
     * End behaviour with the intake running. MIDDLE uses the RIGHT-authored path (mirrored for the
     * left side); the depot paths are LEFT-authored (depot lives on the left), so never mirrored.
     */
    public static Command endBehaviour(EndBehaviour endBehaviour, boolean isLeft) {
        return switch (endBehaviour) {
            case NONE -> Commands.none();
            case MIDDLE -> Commands.deadline(followPathFile("RightEndtoMiddle", isLeft), intake());
            case DEPOT -> Commands.deadline(followPathFile("LeftDriveDepot", false), intake());
            case DEPOT_DRIVE_THROUGH ->
                    Commands.deadline(
                            followPathFile("LeftDriveDepotDriveThrough", false), intake());
        };
    }
}
