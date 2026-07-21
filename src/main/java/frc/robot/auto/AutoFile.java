package frc.robot.auto;

import static frc.robot.auto.AutoActions.*;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldPublisher;
import frc.robot.RobotConstants;
import frc.robot.auto.AutoRoutines.EndBehaviour;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import lib.ironpulse.utils.AllianceFlipUtil;
import lombok.Getter;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Autonomous selector, ported from the competition robot's {@code AutoFile}.
 *
 * <p>Choosers (in order): Side, Waiting, Start Behaviour, Second Sweep, Sweep Times, End Behaviour.
 * {@link #buildAuto()} maps them onto {@link AutoRoutines#competitionAuto}.
 *
 * <p><b>Right-side rule:</b> depot end behaviours (Depot / DepotDriveThrough) only exist on the
 * left; if Side = RIGHT they are forced back to None.
 */
public class AutoFile {
    public static final String SHOW_PREVIEW_KEY = "Auto/Show Preview";
    public static final String SHOOT_TEST_KEY = "Auto/Shoot Test";

    @Getter
    private static final LoggedDashboardChooser<Side> sideChooser =
            new LoggedDashboardChooser<>("Auto/Side");

    private static final LoggedDashboardChooser<Integer> waitingChooser =
            new LoggedDashboardChooser<>("Auto/Waiting");
    private static final LoggedDashboardChooser<StartBehaviour> startBehaviourChooser =
            new LoggedDashboardChooser<>("Auto/Start Behaviour");
    private static final LoggedDashboardChooser<SecondSweepBehaviour> secondSweepChooser =
            new LoggedDashboardChooser<>("Auto/Second Sweep");
    private static final LoggedDashboardChooser<Integer> sweepTimesChooser =
            new LoggedDashboardChooser<>("Auto/Sweep Times");
    private static final LoggedDashboardChooser<EndBehaviour> endBehaviourChooser =
            new LoggedDashboardChooser<>("Auto/End Behaviour");

    private static final Alert rightSideDepotAlert =
            new Alert(
                    "Depot end behaviour is left-only; forcing None on the right side.",
                    Alert.AlertType.kWarning);
    private static final Alert previewAlert =
            new Alert("Failed to build selected auto preview.", Alert.AlertType.kError);
    private static String lastPreviewSelection = "";

    private static <E extends Enum<E>> void initChooser(
            LoggedDashboardChooser<E> chooser, E[] values, E defaultValue) {
        chooser.addDefaultOption(defaultValue.toString(), defaultValue);
        for (E e : values) {
            if (e != defaultValue) {
                chooser.addOption(e.toString(), e);
            }
        }
    }

    public static void init() {
        validatePaths();
        SmartDashboard.setDefaultBoolean(SHOW_PREVIEW_KEY, true);
        SmartDashboard.setDefaultBoolean(SHOOT_TEST_KEY, false);
        if (!RobotConstants.ENABLE_NT_PARAMS) {
            SmartDashboard.putBoolean(SHOOT_TEST_KEY, false);
        }
        // Default config: Trench start, Middle second sweep, 2 sweeps, no end behaviour.
        initChooser(sideChooser, Side.values(), Side.RIGHT);
        initChooser(startBehaviourChooser, StartBehaviour.values(), StartBehaviour.TRENCH_START);
        initChooser(secondSweepChooser, SecondSweepBehaviour.values(), SecondSweepBehaviour.MIDDLE);
        initChooser(endBehaviourChooser, EndBehaviour.values(), EndBehaviour.NONE);

        // Pre-auto delay in whole seconds (0 = start immediately).
        waitingChooser.addDefaultOption("0", 0);
        waitingChooser.addOption("1", 1);
        waitingChooser.addOption("2", 2);
        waitingChooser.addOption("3", 3);

        sweepTimesChooser.addDefaultOption("2", 2);
        sweepTimesChooser.addOption("1", 1);
    }

    /** Fail fast at boot if a referenced path file is missing / malformed. */
    private static void validatePaths() {
        File dir = new File(Filesystem.getDeployDirectory(), "pathplanner/paths");
        File[] files = dir.listFiles();
        if (files == null) {
            return;
        }
        for (File file : files) {
            try {
                PathPlannerPath.fromPathFile(file.getName().replaceFirst("[.][^.]+$", ""));
            } catch (IOException | ParseException e) {
                throw new IllegalArgumentException("Failed to parse path: " + file.getName(), e);
            }
        }
    }

    public static String selectionSummary() {
        return "Side="
                + sideChooser.get()
                + ", Wait="
                + waitingChooser.get()
                + ", Start="
                + startBehaviourChooser.get()
                + ", SecondSweep="
                + secondSweepChooser.get()
                + ", Sweeps="
                + sweepTimesChooser.get()
                + ", End="
                + endBehaviourChooser.get()
                + ", ShootTest="
                + shootTestEnabled();
    }

    public static Command buildAuto() {
        if (shootTestEnabled()) {
            rightSideDepotAlert.set(false);
            return AutoRoutines.shootTestAuto();
        }

        Side side = orDefault(sideChooser.get(), Side.RIGHT);
        int waitSeconds = waitingChooser.get() != null ? waitingChooser.get() : 0;
        StartBehaviour start = orDefault(startBehaviourChooser.get(), StartBehaviour.TRENCH_START);
        if (start == StartBehaviour.SHOOT) {
            rightSideDepotAlert.set(false);
            return AutoRoutines.shootOnlyAuto(waitSeconds);
        }
        SecondSweepBehaviour secondSweep =
                orDefault(secondSweepChooser.get(), SecondSweepBehaviour.MIDDLE);
        int sweepTimes = sweepTimesChooser.get() != null ? sweepTimesChooser.get() : 2;
        EndBehaviour end = orDefault(endBehaviourChooser.get(), EndBehaviour.NONE);

        boolean isLeft = side == Side.LEFT;

        // Right-side rule: depot end behaviours are left-only (MIDDLE and NONE are fine on both).
        boolean isDepot = end == EndBehaviour.DEPOT || end == EndBehaviour.DEPOT_DRIVE_THROUGH;
        if (!isLeft && isDepot) {
            end = EndBehaviour.NONE;
            rightSideDepotAlert.set(true);
        } else {
            rightSideDepotAlert.set(false);
        }

        Pose2d blueStartPose = startPose(start, isLeft);
        Pose2d blueSecondSweepStart = isLeft ? kSecondSweepStartL : kSecondSweepStartR;

        return AutoRoutines.competitionAuto(
                isLeft,
                waitSeconds,
                start == StartBehaviour.BUMP_START,
                blueStartPose,
                startPath(start),
                blueSecondSweepStart,
                secondSweepPath(secondSweep),
                sweepTimes,
                end,
                secondSweep == SecondSweepBehaviour.BUMP_AGAIN,
                startPath(StartBehaviour.BUMP_START));
    }

    /** Refresh the disabled-time Field2d preview when a chooser or alliance selection changes. */
    public static void updatePreview() {
        boolean showPreview = SmartDashboard.getBoolean(SHOW_PREVIEW_KEY, true);
        boolean shootTest = shootTestEnabled();
        String previewSelection =
                selectionSummary()
                        + ", Flipped="
                        + AllianceFlipUtil.shouldFlip()
                        + ", ShowPreview="
                        + showPreview;
        if (previewSelection.equals(lastPreviewSelection)) {
            return;
        }
        lastPreviewSelection = previewSelection;

        if (!showPreview || shootTest) {
            FieldPublisher.setPreview(List.of());
            previewAlert.set(false);
            return;
        }

        Side side = orDefault(sideChooser.get(), Side.RIGHT);
        StartBehaviour start = orDefault(startBehaviourChooser.get(), StartBehaviour.TRENCH_START);
        SecondSweepBehaviour secondSweep =
                orDefault(secondSweepChooser.get(), SecondSweepBehaviour.MIDDLE);
        int sweepTimes = sweepTimesChooser.get() != null ? sweepTimesChooser.get() : 2;
        EndBehaviour end = orDefault(endBehaviourChooser.get(), EndBehaviour.NONE);
        boolean isLeft = side == Side.LEFT;

        if (start == StartBehaviour.SHOOT) {
            FieldPublisher.setPreview(List.of());
            previewAlert.set(false);
            return;
        }

        if (!isLeft && (end == EndBehaviour.DEPOT || end == EndBehaviour.DEPOT_DRIVE_THROUGH)) {
            end = EndBehaviour.NONE;
        }

        try {
            List<Pose2d> preview = new ArrayList<>();
            appendPose(preview, alliancePose(startPose(start, isLeft)));

            if (start == StartBehaviour.BUMP_START) {
                appendPose(preview, alliancePose(isLeft ? kSlopeFrontL : kSlopeFrontR));
            } else {
                appendPath(preview, isLeft ? "LeftTrenchToMiddle" : "RightTrenchToMiddle", false);
            }
            appendPath(preview, startPath(start), isLeft);
            appendPose(preview, alliancePose(isLeft ? kSlopeEndL : kSlopeEndR));

            if (sweepTimes >= 2) {
                if (secondSweep == SecondSweepBehaviour.BUMP_AGAIN) {
                    appendPose(preview, alliancePose(isLeft ? kBumpStartL : kBumpStartR));
                    appendPose(preview, alliancePose(isLeft ? kSlopeFrontL : kSlopeFrontR));
                    appendPath(preview, startPath(StartBehaviour.BUMP_START), isLeft);
                } else {
                    appendPose(
                            preview,
                            alliancePose(isLeft ? kSecondSweepStartL : kSecondSweepStartR));
                    appendPath(preview, secondSweepPath(secondSweep), isLeft);
                }
                appendPose(preview, alliancePose(isLeft ? kSlopeEndL : kSlopeEndR));
            }

            switch (end) {
                case MIDDLE -> appendPath(preview, "RightEndtoMiddle", isLeft);
                case DEPOT -> appendPath(preview, "LeftDriveDepot", false);
                case DEPOT_DRIVE_THROUGH ->
                        appendPath(preview, "LeftDriveDepotDriveThrough", false);
                case NONE -> {
                    // No final movement to preview.
                }
            }

            FieldPublisher.setPreview(preview);
            previewAlert.set(false);
        } catch (IOException | ParseException e) {
            FieldPublisher.setPreview(List.of());
            previewAlert.setText("Failed to build selected auto preview: " + e.getMessage());
            previewAlert.set(true);
        }
    }

    private static boolean shootTestEnabled() {
        return RobotConstants.ENABLE_NT_PARAMS && SmartDashboard.getBoolean(SHOOT_TEST_KEY, false);
    }

    private static void appendPath(List<Pose2d> preview, String pathName, boolean shouldMirror)
            throws IOException, ParseException {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        path = shouldMirror ? path.mirrorPath() : path;
        if (AllianceFlipUtil.shouldFlip()) {
            path = path.flipPath();
        }
        for (Pose2d pose : path.getPathPoses()) {
            appendPose(preview, pose);
        }
    }

    private static Pose2d alliancePose(Pose2d bluePose) {
        return AllianceFlipUtil.apply(bluePose);
    }

    private static void appendPose(List<Pose2d> preview, Pose2d pose) {
        if (preview.isEmpty()
                || preview.get(preview.size() - 1)
                                .getTranslation()
                                .getDistance(pose.getTranslation())
                        > 1e-3) {
            preview.add(pose);
        }
    }

    // ---- chooser -> path/pose mapping (paths are RIGHT-authored; left mirrors them) ----

    private static String startPath(StartBehaviour start) {
        return switch (start) {
            case TRENCH_START -> "RightTrenchStart";
            case TRENCH_START_SHORT -> "RightTrenchStartShort";
            case BUMP_START -> "RightBumpStart";
            case SHOOT -> throw new IllegalArgumentException("SHOOT does not use a start path");
        };
    }

    private static Pose2d startPose(StartBehaviour start, boolean isLeft) {
        return switch (start) {
                // Bump start begins at its own 45-deg pose, then drives past the slope into
                // neutral.
            case BUMP_START -> isLeft ? kBumpStartL : kBumpStartR;
            case TRENCH_START, TRENCH_START_SHORT -> isLeft ? kTrenchStartL : kTrenchStartR;
            case SHOOT -> throw new IllegalArgumentException("SHOOT does not use a start pose");
        };
    }

    private static String secondSweepPath(SecondSweepBehaviour secondSweep) {
        return switch (secondSweep) {
            case MIDDLE -> "RightTrenchSecondMiddle";
            case CROSS -> "RightTrenchSecondCross";
            case AVOID -> "RightTrenchSecondAvoid";
                // Not used for BUMP_AGAIN (it runs the bump cycle, not a trench path); kept so the
                // switch stays exhaustive.
            case BUMP_AGAIN -> startPath(StartBehaviour.BUMP_START);
        };
    }

    private static <E> E orDefault(E value, E fallback) {
        return value != null ? value : fallback;
    }

    private enum Side {
        LEFT,
        RIGHT
    }

    private enum StartBehaviour {
        BUMP_START,
        TRENCH_START,
        TRENCH_START_SHORT,
        SHOOT
    }

    private enum SecondSweepBehaviour {
        MIDDLE,
        CROSS,
        AVOID,
        // Repeats the full bump cycle for the second sweep instead of a trench path.
        BUMP_AGAIN
    }
}
