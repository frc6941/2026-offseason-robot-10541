package frc.robot.auto;

import static frc.robot.auto.AutoActions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
 *   <li>Run the <b>second sweep</b> path directly from the shoot pose — its start waypoint is the
 *       first-sweep shoot pose ({@code kSlopeEnd}), so there is no reposition leg — with the intake
 *       on, come back, aim + shoot again; only when sweep count is 2.
 *   <li>Run the <b>end behaviour</b> (depot paths) with the intake running.
 * </ol>
 */
public class AutoRoutines {
    private static final double SHOOT_ONLY_FEED_SECONDS = 3.0;
    public static final double INTAKE_ZERO_TO_AUTO_PATH_DELAY_SECONDS = 0.5;
    // Hub-touch shoot: back off the hub toward our alliance wall this far before firing, for
    // shooter/hood clearance. Tune to whatever gap the shot needs.
    private static final double HUB_BACKUP_DISTANCE_METERS = 0.4;

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
            // Per-side files (not the auto-mirror): the two sides use non-mirror-symmetric
            // start headings (right 0 deg, left 180 deg) so the intake faces the driver station
            // and the robot holds a constant heading through the trench. shouldMirror=false.
            steps.add(followPathFile(isLeft ? "LeftTrenchToMiddle" : "RightTrenchToMiddle", false));
            steps.add(setSwerveLimitDefault());
        }

        // 1. The opening drive may overlap intake homing, but collection must not start until the
        // pivot has a valid zero. Otherwise finishPivotZeroing() could overwrite the intake mode.
        steps.add(Commands.waitUntil(intake::isPivotZeroed));

        // First sweep: collect out, drive back, aim + shoot.
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
                // No reposition leg: the second-sweep path's start waypoint IS the first-sweep
                // shoot pose (kSlopeEnd), so it flows straight from the shot into the sweep. The
                // drive-back after this sweep holds -90 deg (blue-right frame; mirrored to +90 on
                // the left) instead of kSlopeEnd's default rotation.
                steps.add(
                        sweepCollectShoot(
                                secondSweepPath,
                                isLeft,
                                Rotation2d.fromDegrees(isLeft ? 90.0 : -90.0)));
            }
        }

        // 3. End behaviour (intake running throughout).
        steps.add(endBehaviour(endBehaviour, isLeft));

        // Start intake and hood hard-stop homing together. The path branch may begin after the
        // configured delay (and after hood homing), while intake homing continues independently.
        // Once the intake reaches zero, followModePivot takes over the same pivot requirement.
        Command delayedPath =
                Commands.sequence(
                        Commands.parallel(
                                shootingSuperstructure.zeroCommand(),
                                Commands.waitSeconds(INTAKE_ZERO_TO_AUTO_PATH_DELAY_SECONDS)),
                        Commands.sequence(steps.toArray(Command[]::new)));
        Command intakeHomingAndControl = intake.zeroCommand().andThen(intake.followModePivot());

        return Commands.deadline(delayedPath, intakeHomingAndControl)
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    /** Zero mechanisms, then run only the exact autonomous shoot sequence without a drive path. */
    public static Command shootTestAuto() {
        return zeroEverything()
                .andThen(Commands.deadline(aimAndShootAtHub(), intake.followModePivot()))
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    /**
     * Zero, optionally wait, back off the hub toward our alliance wall for shooter clearance, shoot
     * for three seconds, then hold the drivetrain stopped. Used when the robot starts touching the
     * hub with only its preload to fire.
     */
    public static Command shootOnlyAuto(int waitSeconds) {
        Command shootSequence =
                Commands.sequence(
                        Commands.waitSeconds(Math.max(0, waitSeconds)),
                        backUpTowardAllianceWall(HUB_BACKUP_DISTANCE_METERS),
                        aimAndShootAtHub(SHOOT_ONLY_FEED_SECONDS),
                        Commands.run(swerve::runStop, swerve));
        return zeroEverything()
                .andThen(Commands.deadline(shootSequence, intake.followModePivot()))
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
                            Commands.deadline(
                                    followPathFile("LeftDriveDepot", false),
                                    intake(),
                                    // Pre-spin during the depot drive so the shot needs no spin-up.
                                    shootingSuperstructure.spinUpForShot()),
                            aimAndShootAtHub());
            case DEPOT_DRIVE_THROUGH ->
                    Commands.sequence(
                            Commands.deadline(
                                    followPathFile("LeftDriveDepotDriveThrough", false),
                                    intake(),
                                    shootingSuperstructure.spinUpForShot()),
                            aimAndShootAtHub());
        };
    }
}
