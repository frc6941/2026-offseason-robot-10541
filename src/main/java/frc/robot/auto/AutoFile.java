package frc.robot.auto;

import static frc.robot.auto.AutoRoutines.*;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.File;
import java.io.IOException;
import lombok.Getter;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Autonomous selector, ported from the competition robot's {@code AutoFile}.
 *
 * <p>Publishes dashboard choosers, and {@link #buildAuto()} assembles the selected routine from
 * {@link AutoRoutines}. Wire it up in {@code RobotContainer}: call {@link #init()} once, then
 * return {@link #buildAuto()} from {@code getAutonomousCommand()}.
 */
public class AutoFile {
    @Getter
    private static final LoggedDashboardChooser<AutoType> autoChooser =
            new LoggedDashboardChooser<>("Auto/Type");

    private static final LoggedDashboardChooser<AutoSide> sideChooser =
            new LoggedDashboardChooser<>("Auto/Side");
    private static final LoggedDashboardChooser<SweepMode> sweepModeChooser =
            new LoggedDashboardChooser<>("Auto/Sweep Mode");

    private static final Alert nullConfigAlert =
            new Alert(
                    "Auto configuration is null, running Commands.none()", Alert.AlertType.kError);

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
        initChooser(autoChooser, AutoType.values(), AutoType.SWEEP_AND_SHOOT);
        initChooser(sideChooser, AutoSide.values(), AutoSide.RIGHT);
        initChooser(sweepModeChooser, SweepMode.values(), SweepMode.NORMAL);
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
        return "Type="
                + autoChooser.get()
                + ", Side="
                + sideChooser.get()
                + ", Sweep="
                + sweepModeChooser.get();
    }

    public static Command buildAuto() {
        AutoType type = autoChooser.get();
        if (type == null) {
            nullConfigAlert.set(true);
            return Commands.none();
        }
        nullConfigAlert.set(false);

        boolean isLeft = sideChooser.get() == AutoSide.LEFT;
        String sweepPath = sweepPathFor(sweepModeChooser.get());

        return switch (type) {
            case DO_NOTHING -> Commands.none();
            case SHOOT_ONLY -> shootPreload();
            case SWEEP_AND_SHOOT -> withZeroing(sweepAndShoot(sweepPath, isLeft));
            case PRELOAD_THEN_SWEEP -> withZeroing(preloadThenSweep(sweepPath, isLeft));
            case DOUBLE_SWEEP -> withZeroing(doubleSweep(sweepPath, isLeft));
            case TEST -> test();
        };
    }

    private static String sweepPathFor(SweepMode mode) {
        if (mode == null) {
            return "sweepRight";
        }
        return switch (mode) {
            case FAST -> "quickSweepRight";
            case NORMAL -> "sweepRight";
            case LONG -> "longSweepRight";
        };
    }

    private enum AutoType {
        DO_NOTHING,
        SHOOT_ONLY,
        SWEEP_AND_SHOOT,
        PRELOAD_THEN_SWEEP,
        DOUBLE_SWEEP,
        TEST
    }

    private enum AutoSide {
        RIGHT,
        LEFT
    }

    private enum SweepMode {
        FAST,
        NORMAL,
        LONG
    }
}
