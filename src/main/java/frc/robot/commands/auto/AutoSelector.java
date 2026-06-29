package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldPublisher;
import lib.ironpulse.utils.AllianceFlipUtil;

public class AutoSelector {
    private final AutoBuilder autoBuilder;
    private final Command driveForwardCommand;

    private final SendableChooser<Routine> routineChooser = new SendableChooser<>();
    private final SendableChooser<DepotAxis> depotAxisChooser = new SendableChooser<>();
    private final SendableChooser<AutoCommands.DepotVisitRound> depotRoundChooser = new SendableChooser<>();
    private final SendableChooser<AutoCommands.NeutralSweepMode> firstMidModeChooser = new SendableChooser<>();
    private final SendableChooser<AutoCommands.NeutralSweepMode> secondMidModeChooser = new SendableChooser<>();
    private final SendableChooser<AutoCommands.NeutralSweepDirection> firstMidDirectionChooser = new SendableChooser<>();
    private final SendableChooser<AutoCommands.NeutralSweepDirection> secondMidDirectionChooser = new SendableChooser<>();
    private final SendableChooser<Side> sideChooser = new SendableChooser<>();
    private final SendableChooser<TargetPoint> targetChooser = new SendableChooser<>();

    public AutoSelector(AutoBuilder autoBuilder, Command driveForwardCommand) {
        this.autoBuilder = autoBuilder;
        this.driveForwardCommand = driveForwardCommand;

        configureChoosers();
        publishChoosers();
    }

    private void configureChoosers() {
        routineChooser.setDefaultOption("Do Nothing", Routine.DO_NOTHING);
        routineChooser.addOption("Drive Forward", Routine.DRIVE_FORWARD);
        routineChooser.addOption("Depot Collect", Routine.DEPOT_COLLECT);
        routineChooser.addOption("Mid Sweep", Routine.MID_TWO_CYCLE);
        routineChooser.addOption("Mid Step Only", Routine.MID_STEP_ONLY);
        routineChooser.addOption("Go To Target", Routine.GO_TO_TARGET);
        routineChooser.addOption("Trench Clear", Routine.TRENCH_CLEAR);
        routineChooser.addOption("Bump Cross", Routine.BUMP_CROSS);
        routineChooser.addOption("Depot Through", Routine.DEPOT_THROUGH);

        depotAxisChooser.setDefaultOption("X", DepotAxis.X);
        depotAxisChooser.addOption("Y", DepotAxis.Y);

        depotRoundChooser.setDefaultOption("No Depot", AutoCommands.DepotVisitRound.NONE);
        depotRoundChooser.addOption("Depot Start", AutoCommands.DepotVisitRound.START);
        depotRoundChooser.addOption("Depot Round 1", AutoCommands.DepotVisitRound.FIRST);
        depotRoundChooser.addOption("Depot Round 2", AutoCommands.DepotVisitRound.SECOND);

        configureMidModeChooser(firstMidModeChooser);
        configureMidModeChooser(secondMidModeChooser);

        configureMidDirectionChooser(firstMidDirectionChooser, AutoCommands.NeutralSweepDirection.LEFT_TO_RIGHT);
        configureMidDirectionChooser(secondMidDirectionChooser, AutoCommands.NeutralSweepDirection.RIGHT_TO_LEFT);

        sideChooser.setDefaultOption("Left", Side.LEFT);
        sideChooser.addOption("Right", Side.RIGHT);

        targetChooser.setDefaultOption("Outpost", TargetPoint.OUTPOST);
        targetChooser.addOption("Hub Center Start", TargetPoint.HUB_CENTER_START);
        targetChooser.addOption("Left Bump Launch", TargetPoint.LEFT_BUMP_LAUNCH);
        targetChooser.addOption("Right Bump Launch", TargetPoint.RIGHT_BUMP_LAUNCH);
        targetChooser.addOption("Left Trench Launch", TargetPoint.LEFT_TRENCH_LAUNCH);
        targetChooser.addOption("Right Trench Launch", TargetPoint.RIGHT_TRENCH_LAUNCH);
        targetChooser.addOption("Left Climb", TargetPoint.LEFT_CLIMB);
        targetChooser.addOption("Right Climb", TargetPoint.RIGHT_CLIMB);
        targetChooser.addOption("Left Tower Through", TargetPoint.LEFT_TOWER_THROUGH);
        targetChooser.addOption("Right Tower Through", TargetPoint.RIGHT_TOWER_THROUGH);
    }

    private void publishChoosers() {
        SmartDashboard.putData("Auto/Routine", routineChooser);
        SmartDashboard.putData("Auto/Depot Axis", depotAxisChooser);
        SmartDashboard.putData("Auto/Depot Round", depotRoundChooser);
        SmartDashboard.putData("Auto/First Mid Mode", firstMidModeChooser);
        SmartDashboard.putData("Auto/Second Mid Mode", secondMidModeChooser);
        SmartDashboard.putData("Auto/First Mid Direction", firstMidDirectionChooser);
        SmartDashboard.putData("Auto/Second Mid Direction", secondMidDirectionChooser);
        SmartDashboard.putData("Auto/Side", sideChooser);
        SmartDashboard.putData("Auto/Target", targetChooser);
    }

    public Command getCommand() {
        String selectionSummary = getSelectionSummary();
        SmartDashboard.putString("Auto/Selected", selectionSummary);
        return buildSelectedCommand().withName(selectionSummary);
    }

    /** Builds the command for the current selection (no withName) — reused for preview capture. */
    private Command buildSelectedCommand() {
        return switch (selectedRoutine()) {
            case DO_NOTHING -> Commands.none();
            case DRIVE_FORWARD -> driveForwardCommand;
            case DEPOT_COLLECT -> selectedDepotAxis() == DepotAxis.X
                    ? autoBuilder.buildDepotXAuto()
                    : autoBuilder.buildDepotYAuto();
            case MID_STEP_ONLY -> autoBuilder.buildNeutralSweepAuto(
                    selectedFirstMidMode(),
                    selectedFirstMidDirection());
            case MID_TWO_CYCLE -> autoBuilder.buildMidTwoCycleAuto(
                    selectedFirstMidMode(),
                    selectedSecondMidMode(),
                    selectedFirstMidDirection(),
                    selectedSecondMidDirection(),
                    selectedDepotAxis(),
                    selectedDepotRound());
            case GO_TO_TARGET -> buildTargetCommand(selectedTarget());
            case TRENCH_CLEAR -> selectedSide() == Side.LEFT
                    ? autoBuilder.buildLeftTrenchClearAuto()
                    : autoBuilder.buildRightTrenchClearAuto();
            case BUMP_CROSS -> selectedSide() == Side.LEFT
                    ? autoBuilder.buildLeftBumpCrossAuto()
                    : autoBuilder.buildRightBumpCrossAuto();
            case DEPOT_THROUGH -> selectedSide() == Side.LEFT
                    ? autoBuilder.buildDepotLeftThroughAuto()
                    : autoBuilder.buildDepotRightThroughAuto();
        };
    }

    private String lastPreviewKey = "";

    public void updateDashboard() {
        String summary = getSelectionSummary();
        SmartDashboard.putString("Auto/Selected", summary);

        // Publish the selected auto's target waypoints to the Field2d for an Elastic preview.
        // Re-capture only when the selection or alliance changes (each capture builds a throwaway
        // command to record its pathfind targets).
        String previewKey = summary + "|" + AllianceFlipUtil.shouldFlip();
        if (!previewKey.equals(lastPreviewKey)) {
            lastPreviewKey = previewKey;
            FieldPublisher.setPreview(captureSelectedTargets());
        }
    }

    /** Builds the selected command under preview capture and returns its alliance-applied waypoints. */
    private java.util.List<Pose2d> captureSelectedTargets() {
        java.util.List<Pose2d> blue = new java.util.ArrayList<>();
        AutoCommands.startPreviewCapture(blue);
        try {
            buildSelectedCommand(); // build-only; records pathfind targets, then discarded
        } catch (RuntimeException e) {
            // best-effort preview — ignore any build hiccup
        } finally {
            AutoCommands.stopPreviewCapture();
        }
        java.util.List<Pose2d> out = new java.util.ArrayList<>(blue.size());
        for (Pose2d p : blue) {
            out.add(AllianceFlipUtil.apply(p));
        }
        return out;
    }

    public String getSelectionSummary() {
        return "Routine=" + selectedRoutine()
                + ", DepotAxis=" + selectedDepotAxis()
                + ", DepotRound=" + selectedDepotRound()
                + ", FirstMidMode=" + selectedFirstMidMode()
                + ", SecondMidMode=" + selectedSecondMidMode()
                + ", FirstMidDirection=" + selectedFirstMidDirection()
                + ", SecondMidDirection=" + selectedSecondMidDirection()
                + ", Side=" + selectedSide()
                + ", Target=" + selectedTarget();
    }

    private Command buildTargetCommand(TargetPoint target) {
        return switch (target) {
            case OUTPOST -> autoBuilder.buildOutpostAuto();
            case HUB_CENTER_START -> autoBuilder.buildHubCenterStartAuto();
            case LEFT_BUMP_LAUNCH -> autoBuilder.buildLeftBumpLaunchAuto();
            case RIGHT_BUMP_LAUNCH -> autoBuilder.buildRightBumpLaunchAuto();
            case LEFT_TRENCH_LAUNCH -> autoBuilder.buildLeftTrenchLaunchAuto();
            case RIGHT_TRENCH_LAUNCH -> autoBuilder.buildRightTrenchLaunchAuto();
            case LEFT_CLIMB -> autoBuilder.buildLeftClimbAuto();
            case RIGHT_CLIMB -> autoBuilder.buildRightClimbAuto();
            case LEFT_TOWER_THROUGH -> autoBuilder.buildLeftTowerThroughAuto();
            case RIGHT_TOWER_THROUGH -> autoBuilder.buildRightTowerThroughAuto();
        };
    }

    private Routine selectedRoutine() {
        Routine selected = routineChooser.getSelected();
        return selected == null ? Routine.DO_NOTHING : selected;
    }

    private DepotAxis selectedDepotAxis() {
        DepotAxis selected = depotAxisChooser.getSelected();
        return selected == null ? DepotAxis.X : selected;
    }

    private void configureMidModeChooser(SendableChooser<AutoCommands.NeutralSweepMode> chooser) {
        chooser.setDefaultOption("Salesman", AutoCommands.NeutralSweepMode.SALESMAN);
        chooser.addOption("Conservative", AutoCommands.NeutralSweepMode.CONSERVATIVE);
        chooser.addOption("Neutral", AutoCommands.NeutralSweepMode.NEUTRAL);
        chooser.addOption("Flightless", AutoCommands.NeutralSweepMode.FLIGHTLESS);
        chooser.addOption("Flightless Wide", AutoCommands.NeutralSweepMode.FLIGHTLESS_WIDE);
        chooser.addOption("Flightless Wave", AutoCommands.NeutralSweepMode.FLIGHTLESS_WAVE);
        chooser.addOption("Davis", AutoCommands.NeutralSweepMode.DAVIS);
        chooser.addOption("Davis Friendship", AutoCommands.NeutralSweepMode.DAVIS_FRIENDSHIP);
        chooser.addOption("Coriolis", AutoCommands.NeutralSweepMode.CORIOLIS);
        chooser.addOption("Salesman Turn", AutoCommands.NeutralSweepMode.SALESMAN_TURN);
        chooser.addOption("Wave", AutoCommands.NeutralSweepMode.WAVE);
    }

    private void configureMidDirectionChooser(
            SendableChooser<AutoCommands.NeutralSweepDirection> chooser,
            AutoCommands.NeutralSweepDirection defaultDirection) {
        if (defaultDirection == AutoCommands.NeutralSweepDirection.LEFT_TO_RIGHT) {
            chooser.setDefaultOption("Left To Right", AutoCommands.NeutralSweepDirection.LEFT_TO_RIGHT);
            chooser.addOption("Right To Left", AutoCommands.NeutralSweepDirection.RIGHT_TO_LEFT);
        } else {
            chooser.setDefaultOption("Right To Left", AutoCommands.NeutralSweepDirection.RIGHT_TO_LEFT);
            chooser.addOption("Left To Right", AutoCommands.NeutralSweepDirection.LEFT_TO_RIGHT);
        }
    }

    private AutoCommands.NeutralSweepMode selectedFirstMidMode() {
        AutoCommands.NeutralSweepMode selected = firstMidModeChooser.getSelected();
        return selected == null ? AutoCommands.NeutralSweepMode.SALESMAN : selected;
    }

    private AutoCommands.NeutralSweepMode selectedSecondMidMode() {
        AutoCommands.NeutralSweepMode selected = secondMidModeChooser.getSelected();
        return selected == null ? AutoCommands.NeutralSweepMode.SALESMAN : selected;
    }

    private AutoCommands.DepotVisitRound selectedDepotRound() {
        AutoCommands.DepotVisitRound selected = depotRoundChooser.getSelected();
        if (selected == null) {
            selected = AutoCommands.DepotVisitRound.NONE;
        }
        return selected;
    }

    private AutoCommands.NeutralSweepDirection selectedFirstMidDirection() {
        AutoCommands.NeutralSweepDirection selected = firstMidDirectionChooser.getSelected();
        return selected == null ? AutoCommands.NeutralSweepDirection.LEFT_TO_RIGHT : selected;
    }

    private AutoCommands.NeutralSweepDirection selectedSecondMidDirection() {
        AutoCommands.NeutralSweepDirection selected = secondMidDirectionChooser.getSelected();
        return selected == null ? AutoCommands.NeutralSweepDirection.RIGHT_TO_LEFT : selected;
    }

    private Side selectedSide() {
        Side selected = sideChooser.getSelected();
        return selected == null ? Side.LEFT : selected;
    }

    private TargetPoint selectedTarget() {
        TargetPoint selected = targetChooser.getSelected();
        return selected == null ? TargetPoint.OUTPOST : selected;
    }

    public enum Routine {
        DO_NOTHING,
        DRIVE_FORWARD,
        DEPOT_COLLECT,
        MID_STEP_ONLY,
        MID_TWO_CYCLE,
        GO_TO_TARGET,
        TRENCH_CLEAR,
        BUMP_CROSS,
        DEPOT_THROUGH
    }

    public enum DepotAxis {
        X,
        Y
    }

    public enum Side {
        LEFT,
        RIGHT
    }

    public enum TargetPoint {
        OUTPOST,
        HUB_CENTER_START,
        LEFT_BUMP_LAUNCH,
        RIGHT_BUMP_LAUNCH,
        LEFT_TRENCH_LAUNCH,
        RIGHT_TRENCH_LAUNCH,
        LEFT_CLIMB,
        RIGHT_CLIMB,
        LEFT_TOWER_THROUGH,
        RIGHT_TOWER_THROUGH
    }
}
