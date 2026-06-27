package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoSelector {
    private final AutoBuilder autoBuilder;
    private final Command driveForwardCommand;

    private final SendableChooser<Routine> routineChooser = new SendableChooser<>();
    private final SendableChooser<DepotAxis> depotAxisChooser = new SendableChooser<>();
    private final SendableChooser<AutoCommands.DepotVisitRound> depotRoundChooser = new SendableChooser<>();
    private final SendableChooser<AutoCommands.NeutralSweepMode> firstMidModeChooser = new SendableChooser<>();
    private final SendableChooser<AutoCommands.NeutralSweepMode> secondMidModeChooser = new SendableChooser<>();
    private final SendableChooser<AutoCommands.NeutralSweepDirection> midDirectionChooser = new SendableChooser<>();
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

        midDirectionChooser.setDefaultOption("Left To Right", AutoCommands.NeutralSweepDirection.LEFT_TO_RIGHT);
        midDirectionChooser.addOption("Right To Left", AutoCommands.NeutralSweepDirection.RIGHT_TO_LEFT);

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
        SmartDashboard.putData("Auto/Mid Direction", midDirectionChooser);
        SmartDashboard.putData("Auto/Side", sideChooser);
        SmartDashboard.putData("Auto/Target", targetChooser);
    }

    public Command getCommand() {
        String selectionSummary = getSelectionSummary();
        SmartDashboard.putString("Auto/Selected", selectionSummary);

        Command command = switch (selectedRoutine()) {
            case DO_NOTHING -> Commands.none();
            case DRIVE_FORWARD -> driveForwardCommand;
            case DEPOT_COLLECT -> selectedDepotAxis() == DepotAxis.X
                    ? autoBuilder.buildDepotXAuto()
                    : autoBuilder.buildDepotYAuto();
            case MID_STEP_ONLY -> autoBuilder.buildNeutralSweepAuto(selectedFirstMidMode(), selectedMidDirection());
            case MID_TWO_CYCLE -> autoBuilder.buildMidTwoCycleAuto(
                    selectedFirstMidMode(),
                    selectedSecondMidMode(),
                    selectedMidDirection(),
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

        return command.withName(selectionSummary);
    }

    public void updateDashboard() {
        SmartDashboard.putString("Auto/Selected", getSelectionSummary());
    }

    public String getSelectionSummary() {
        return "Routine=" + selectedRoutine()
                + ", DepotAxis=" + selectedDepotAxis()
                + ", DepotRound=" + selectedDepotRound()
                + ", FirstMidMode=" + selectedFirstMidMode()
                + ", SecondMidMode=" + selectedSecondMidMode()
                + ", MidDirection=" + selectedMidDirection()
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
        chooser.addOption("Davis", AutoCommands.NeutralSweepMode.DAVIS);
        chooser.addOption("Davis Friendship", AutoCommands.NeutralSweepMode.DAVIS_FRIENDSHIP);
        chooser.addOption("Coriolis", AutoCommands.NeutralSweepMode.CORIOLIS);
        chooser.addOption("Salesman Turn", AutoCommands.NeutralSweepMode.SALESMAN_TURN);
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

    private AutoCommands.NeutralSweepDirection selectedMidDirection() {
        AutoCommands.NeutralSweepDirection selected = midDirectionChooser.getSelected();
        return selected == null ? AutoCommands.NeutralSweepDirection.LEFT_TO_RIGHT : selected;
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
