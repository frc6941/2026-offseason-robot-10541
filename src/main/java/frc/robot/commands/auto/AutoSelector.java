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
    private final SendableChooser<AutoCommands.MidKind> firstMidKindChooser = new SendableChooser<>();
    private final SendableChooser<AutoCommands.MidKind> secondMidKindChooser = new SendableChooser<>();
    private final SendableChooser<AutoCommands.NeutralSweepDirection> firstMidDirectionChooser = new SendableChooser<>();
    private final SendableChooser<AutoCommands.NeutralSweepDirection> secondMidDirectionChooser = new SendableChooser<>();
    private final SendableChooser<Side> firstShootPositionChooser = new SendableChooser<>();
    private final SendableChooser<Side> secondShootPositionChooser = new SendableChooser<>();
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
        configureMidKindChooser(firstMidKindChooser);
        configureMidKindChooser(secondMidKindChooser);

        configureMidDirectionChooser(firstMidDirectionChooser, AutoCommands.NeutralSweepDirection.LEFT_TO_RIGHT);
        configureMidDirectionChooser(secondMidDirectionChooser, AutoCommands.NeutralSweepDirection.RIGHT_TO_LEFT);

        configureShootPositionChooser(firstShootPositionChooser, Side.RIGHT);
        configureShootPositionChooser(secondShootPositionChooser, Side.LEFT);

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
        SmartDashboard.putData("Auto/First Mid Kind", firstMidKindChooser);
        SmartDashboard.putData("Auto/Second Mid Kind", secondMidKindChooser);
        SmartDashboard.putData("Auto/First Mid Direction", firstMidDirectionChooser);
        SmartDashboard.putData("Auto/Second Mid Direction", secondMidDirectionChooser);
        SmartDashboard.putData("Auto/First Shoot Position", firstShootPositionChooser);
        SmartDashboard.putData("Auto/Second Shoot Position", secondShootPositionChooser);
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
                    selectedFirstMidDirection(),
                    selectedFirstMidKind());
            case MID_TWO_CYCLE -> autoBuilder.buildMidTwoCycleAuto(
                    selectedFirstMidMode(),
                    selectedSecondMidMode(),
                    selectedFirstMidDirection(),
                    selectedSecondMidDirection(),
                    selectedFirstMidKind(),
                    selectedSecondMidKind(),
                    selectedFirstShootPosition(),
                    selectedSecondShootPosition(),
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
                + ", FirstMidMode=" + midModeLabel(selectedFirstMidMode())
                + ", SecondMidMode=" + midModeLabel(selectedSecondMidMode())
                + ", FirstMidKind=" + selectedFirstMidKind()
                + ", SecondMidKind=" + selectedSecondMidKind()
                + ", FirstMidDirection=" + selectedFirstMidDirection()
                + ", SecondMidDirection=" + selectedSecondMidDirection()
                + ", FirstShootPosition=" + selectedFirstShootPosition()
                + ", SecondShootPosition=" + selectedSecondShootPosition()
                + ", Side=" + selectedSide()
                + ", Target=" + selectedTarget();
    }

    private String midModeLabel(AutoCommands.NeutralSweepMode mode) {
        return switch (mode) {
            case CONSERVATIVE -> "Safe Inner Sweep";
            case NEUTRAL -> "Center Line Sweep";
            case FLIGHTLESS -> "Near Tower Sweep";
            case FLIGHTLESS_WIDE -> "Near Tower Wide Sweep";
            case FLIGHTLESS_WAVE -> "Near Tower Wave Sweep";
            case DAVIS -> "Far Edge Sweep";
            case DAVIS_FRIENDSHIP -> "Far Edge + Center Sweep";
            case CORIOLIS -> "Back Center Sweep";
            case CENTER_FORWARD -> "Mid-Back Center Sweep";
            case SALESMAN -> "Full Pool Sweep";
            case SALESMAN_TURN -> "Full Pool Turn-In Sweep";
            case WAVE -> "Center Wave Sweep";
        };
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
        chooser.setDefaultOption("Full Pool Sweep", AutoCommands.NeutralSweepMode.SALESMAN);
        chooser.addOption("Safe Inner Sweep", AutoCommands.NeutralSweepMode.CONSERVATIVE);
        chooser.addOption("Center Line Sweep", AutoCommands.NeutralSweepMode.NEUTRAL);
        chooser.addOption("Near Tower Sweep", AutoCommands.NeutralSweepMode.FLIGHTLESS);
        chooser.addOption("Near Tower Wide Sweep", AutoCommands.NeutralSweepMode.FLIGHTLESS_WIDE);
        chooser.addOption("Near Tower Wave Sweep", AutoCommands.NeutralSweepMode.FLIGHTLESS_WAVE);
        chooser.addOption("Far Edge Sweep", AutoCommands.NeutralSweepMode.DAVIS);
        chooser.addOption("Far Edge + Center Sweep", AutoCommands.NeutralSweepMode.DAVIS_FRIENDSHIP);
        chooser.addOption("Back Center Sweep", AutoCommands.NeutralSweepMode.CORIOLIS);
        chooser.addOption("Mid-Back Center Sweep", AutoCommands.NeutralSweepMode.CENTER_FORWARD);
        chooser.addOption("Full Pool Turn-In Sweep", AutoCommands.NeutralSweepMode.SALESMAN_TURN);
        chooser.addOption("Center Wave Sweep", AutoCommands.NeutralSweepMode.WAVE);
    }

    private void configureMidKindChooser(SendableChooser<AutoCommands.MidKind> chooser) {
        chooser.setDefaultOption("Full", AutoCommands.MidKind.FULL);
        chooser.addOption("Half", AutoCommands.MidKind.HALF);
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

    private void configureShootPositionChooser(SendableChooser<Side> chooser, Side defaultPosition) {
        if (defaultPosition == Side.LEFT) {
            chooser.setDefaultOption("Left", Side.LEFT);
            chooser.addOption("Right", Side.RIGHT);
        } else {
            chooser.setDefaultOption("Right", Side.RIGHT);
            chooser.addOption("Left", Side.LEFT);
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

    private AutoCommands.MidKind selectedFirstMidKind() {
        AutoCommands.MidKind selected = firstMidKindChooser.getSelected();
        return selected == null ? AutoCommands.MidKind.FULL : selected;
    }

    private AutoCommands.MidKind selectedSecondMidKind() {
        AutoCommands.MidKind selected = secondMidKindChooser.getSelected();
        return selected == null ? AutoCommands.MidKind.FULL : selected;
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

    private Side selectedFirstShootPosition() {
        Side selected = firstShootPositionChooser.getSelected();
        return selected == null ? Side.RIGHT : selected;
    }

    private Side selectedSecondShootPosition() {
        Side selected = secondShootPositionChooser.getSelected();
        return selected == null ? Side.LEFT : selected;
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
