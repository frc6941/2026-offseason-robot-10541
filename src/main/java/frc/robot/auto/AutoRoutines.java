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
 *   <li>Trench starts (not bump) only: dash to the middle at an unlimited speed cap via {@code
 *       RightTrenchToMiddle}, then drop back to the default limit.
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
            int waitSeconds,
            boolean startFromBump,
            Pose2d blueStartPose,
            String startPath,
            Pose2d blueSecondSweepStart,
            String secondSweepPath,
            int sweepTimes,
            EndBehaviour endBehaviour,
            boolean secondRunBumpAgain,
            String bumpPath) {
        List<Command> steps = new ArrayList<>();

        // 0. Optional pre-auto delay (e.g. to let alliance partners clear the field first).
        if (waitSeconds > 0) {
            steps.add(Commands.waitSeconds(waitSeconds));
        }

        // 0. Sim-only pose reset so the robot starts where the path expects.
        steps.add(resetOnPose(blueStartPose));

        // 0a. Bump start begins on the bump — cross out into the neutral zone before anything else.
        if (startFromBump) {
            steps.add(drivePastSlope(isLeft, true));
        }

        // 0b. Trench starts (not bump) open with a fast unguarded dash to the middle before the
        // actual trench sweep: lift the speed cap, run RightTrenchToMiddle, then drop back to the
        // default limit so the sweep path itself runs at normal speed.
        if (!startFromBump) {
            steps.add(setSwerveLimitUnlimited());
            steps.add(followPathFile("RightTrenchToMiddle", isLeft));
            steps.add(setSwerveLimitDefault());
        }

        // 1. First sweep: collect out, drive back, aim + shoot. (Pivot homing runs in parallel with
        // the whole routine — see the deadline at the end.)
        steps.add(sweepCollectShoot(startPath, isLeft));

        // 2. Optional second sweep. BUMP_AGAIN repeats the bump cycle so it can follow any first
        // attempt (trench or bump) without repositioning to the trench start: reposition to the
        // bump-start pose first (the first sweep ends at kSlopeEnd, not kBumpStart), then cross the
        // bump the normal way (holding the bump-start heading), then follow the bumpstart path,
        // drive back, aim + shoot. Otherwise reposition to the hardcoded start and run the trench
        // second-sweep path.
        if (sweepTimes >= 2) {
            if (secondRunBumpAgain) {
                steps.add(driveToPose(AllianceFlipUtil.apply(isLeft ? kBumpStartL : kBumpStartR)));
                steps.add(drivePastSlope(isLeft, true));
                steps.add(sweepCollectShoot(bumpPath, isLeft));
            } else {
                // Reposition to the second-sweep start with Autopilot (straight beeline) rather
                // than the pose PID — the alliance-side lane here is clear after the shot.
                steps.add(driveToPoseAutoPilot(AllianceFlipUtil.apply(blueSecondSweepStart)));
                steps.add(sweepCollectShoot(secondSweepPath, isLeft));
            }
        }

        // 3. End behaviour (intake running throughout).
        steps.add(endBehaviour(endBehaviour, isLeft));

        // Run the routine with a pivot manager in parallel: home the pivot once, then continuously
        // drive it to the current intake mode's angle. The routine holds the pivot requirement (via
        // this same zeroCommand), which suppresses the pivot's default command for the whole auto —
        // so without this, intake()/retractIntake() mode changes would flip the mode but never move
        // the pivot. Homing runs alongside the opening steps (home on the move), and the manager
        // makes every deploy/retract in the sweeps actually actuate. The routine is the deadline,
        // so
        // when it ends the manager stops.
        return Commands.deadline(
                        Commands.sequence(steps.toArray(Command[]::new)),
                        Commands.sequence(intake.zeroCommand(), intake.followModePivot()))
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
                // Depot options: drive/collect the depot, then aim at the hub and empty the hopper.
            case DEPOT ->
                    Commands.sequence(
                            Commands.deadline(followPathFile("LeftDriveDepot", false), intake()),
                            aimAndShootAtHub());
            case DEPOT_DRIVE_THROUGH ->
                    Commands.sequence(
                            Commands.deadline(
                                    followPathFile("LeftDriveDepotDriveThrough", false), intake()),
                            aimAndShootAtHub());
        };
    }
}
