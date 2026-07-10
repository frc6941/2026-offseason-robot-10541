package frc.robot.auto;

import static frc.robot.auto.AutoActions.*;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoRoutines.EndBehaviour;
import java.io.File;
import java.io.IOException;
import lombok.Getter;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Autonomous selector, ported from the competition robot's {@code AutoFile}.
 *
 * <p>Choosers (in order): Side, Start Behaviour, Second Sweep, Sweep Times, End Behaviour. {@link
 * #buildAuto()} maps them onto {@link AutoRoutines#competitionAuto}.
 *
 * <p><b>Right-side rule:</b> depot end behaviours (Depot / DepotDriveThrough) only exist on the
 * left; if Side = RIGHT they are forced back to None.
 */
public class AutoFile {
    @Getter
    private static final LoggedDashboardChooser<Side> sideChooser =
            new LoggedDashboardChooser<>("Auto/Side");

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
        // Default config: Trench start, Middle second sweep, 2 sweeps, no end behaviour.
        initChooser(sideChooser, Side.values(), Side.RIGHT);
        initChooser(startBehaviourChooser, StartBehaviour.values(), StartBehaviour.TRENCH_START);
        initChooser(secondSweepChooser, SecondSweepBehaviour.values(), SecondSweepBehaviour.MIDDLE);
        initChooser(endBehaviourChooser, EndBehaviour.values(), EndBehaviour.NONE);

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
                + ", Start="
                + startBehaviourChooser.get()
                + ", SecondSweep="
                + secondSweepChooser.get()
                + ", Sweeps="
                + sweepTimesChooser.get()
                + ", End="
                + endBehaviourChooser.get();
    }

    public static Command buildAuto() {
        Side side = orDefault(sideChooser.get(), Side.RIGHT);
        StartBehaviour start = orDefault(startBehaviourChooser.get(), StartBehaviour.TRENCH_START);
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
                start == StartBehaviour.BUMP_START,
                blueStartPose,
                startPath(start),
                blueSecondSweepStart,
                secondSweepPath(secondSweep),
                sweepTimes,
                end);
    }

    // ---- chooser -> path/pose mapping (paths are RIGHT-authored; left mirrors them) ----

    private static String startPath(StartBehaviour start) {
        return switch (start) {
            case TRENCH_START -> "RightTrenchStart";
            case TRENCH_START_SHORT -> "RightTrenchStartShort";
            case BUMP_START -> "RightBumpStart";
        };
    }

    private static Pose2d startPose(StartBehaviour start, boolean isLeft) {
        return switch (start) {
                // Bump start begins at its own 45-deg pose, then drives past the slope into
                // neutral.
            case BUMP_START -> isLeft ? kBumpStartL : kBumpStartR;
            case TRENCH_START, TRENCH_START_SHORT -> isLeft ? kTrenchStartL : kTrenchStartR;
        };
    }

    private static String secondSweepPath(SecondSweepBehaviour secondSweep) {
        return switch (secondSweep) {
            case MIDDLE -> "RightTrenchSecondMiddle";
            case CROSS -> "RightTrenchSecondCross";
            case AVOID -> "RightTrenchSecondAvoid";
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
        TRENCH_START_SHORT
    }

    private enum SecondSweepBehaviour {
        MIDDLE,
        CROSS,
        AVOID
    }
}
