package frc.robot.commands.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.FieldConstants;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class GuidedPathPlannerTest {
    private static final PathConstraints CONSTRAINTS =
            new PathConstraints(4.0, 3.0, Math.toRadians(360.0), Math.toRadians(540.0));

    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @Test
    void generatesFiniteContinuousTrajectoryThroughOrderedGuides() throws Exception {
        Translation2d start = new Translation2d(7.0, 2.0);
        List<GuidedPathPlanner.GuidePoint> guides =
                List.of(
                        new GuidedPathPlanner.GuidePoint(
                                new Translation2d(8.0, 3.0), Rotation2d.fromDegrees(35.0)),
                        new GuidedPathPlanner.GuidePoint(
                                new Translation2d(9.0, 2.0), Rotation2d.fromDegrees(100.0)));

        PathPlannerPath path = GuidedPathPlanner.plan(start, guides, CONSTRAINTS, 2.0, List.of());

        assertTrue(path.getPoint(0).position.getDistance(start) < 1e-6);
        assertFalse(path.getPoint(0).rotationTarget != null);
        assertEquals(100.0, path.getGoalEndState().rotation().getDegrees(), 1e-6);
        assertEquals(2.0, path.getGoalEndState().velocityMPS(), 1e-6);
        int searchStart = 0;
        for (GuidedPathPlanner.GuidePoint guide : guides) {
            int nearestIndex = searchStart;
            double nearest = path.getPoint(nearestIndex).position.getDistance(guide.position());
            for (int i = searchStart + 1; i < path.numPoints(); i++) {
                double distance = path.getPoint(i).position.getDistance(guide.position());
                if (distance < nearest) {
                    nearest = distance;
                    nearestIndex = i;
                }
            }
            assertTrue(nearest < 0.25, "Path missed ordered guide point by " + nearest + " m");
            assertTrue(nearestIndex >= searchStart);
            searchStart = nearestIndex;
        }

        RobotConfig robotConfig = RobotConfig.fromGUISettings();
        assertEquals(46.0, robotConfig.massKG, 1e-6);
        assertEquals(8.185, robotConfig.MOI, 1e-6);
        var trajectory =
                path.generateTrajectory(new ChassisSpeeds(), Rotation2d.kZero, robotConfig);
        assertTrue(Double.isFinite(trajectory.getTotalTimeSeconds()));
        assertTrue(trajectory.getTotalTimeSeconds() > 0.0);
        assertTrue(
                trajectory.getStates().stream()
                                .mapToDouble(state -> state.linearVelocity)
                                .max()
                                .orElseThrow()
                        >= 2.0);
        assertTrue(
                trajectory.getStates().stream()
                        .allMatch(
                                state ->
                                        Double.isFinite(state.timeSeconds)
                                                && Double.isFinite(state.linearVelocity)
                                                && Double.isFinite(state.pose.getX())
                                                && Double.isFinite(state.pose.getY())
                                                && Double.isFinite(
                                                        state.pose.getRotation().getRadians())));
        assertEquals(100.0, trajectory.getEndState().pose.getRotation().getDegrees(), 1e-6);
    }

    @Test
    void wavePathHasNoRotationTargetAtZeroAndGeneratesFiniteTrajectory() throws Exception {
        for (AutoCommands.NeutralSweepDirection direction :
                AutoCommands.NeutralSweepDirection.values()) {
            for (double endProgress : List.of(0.5, 1.0)) {
                PathPlannerPath path =
                        AutoCommands.buildWaveSweepPath(
                                AutoPoints.NeutralZone.LEFT_CENTER,
                                AutoPoints.NeutralZone.RIGHT_CENTER,
                                direction,
                                AutoPoints.NeutralZone.WAVE_AMPLITUDE_METERS,
                                endProgress);

                assertNull(path.getPoint(0).rotationTarget);
                assertFiniteTrajectory(
                        path,
                        path.getIdealStartingState().rotation(),
                        RobotConfig.fromGUISettings());
            }
        }
    }

    @Test
    void allPointBasedMidModesGenerateFiniteTrajectoriesFromBothLaunchSides() throws Exception {
        RobotConfig robotConfig = RobotConfig.fromGUISettings();
        List<Translation2d> starts =
                List.of(
                        AutoPoints.Launch.LEFT_BUMP.getTranslation(),
                        AutoPoints.Launch.RIGHT_BUMP.getTranslation());

        for (AutoCommands.NeutralSweepMode mode : AutoCommands.NeutralSweepMode.values()) {
            if (mode == AutoCommands.NeutralSweepMode.WAVE
                    || mode == AutoCommands.NeutralSweepMode.FLIGHTLESS_WAVE) {
                continue;
            }
            for (AutoCommands.NeutralSweepDirection direction :
                    AutoCommands.NeutralSweepDirection.values()) {
                Rotation2d heading =
                        direction == AutoCommands.NeutralSweepDirection.LEFT_TO_RIGHT
                                ? Rotation2d.fromDegrees(-90.0)
                                : Rotation2d.fromDegrees(90.0);
                List<GuidedPathPlanner.GuidePoint> guides =
                        Arrays.stream(AutoCommands.neutralSweepPoints(mode, direction))
                                .map(point -> new GuidedPathPlanner.GuidePoint(point, heading))
                                .toList();
                for (Translation2d start : starts) {
                    PathPlannerPath path =
                            GuidedPathPlanner.plan(
                                    start,
                                    guides,
                                    AutoCommands.INTAKE_MEDIUM_CONSTRAINTS,
                                    3.0,
                                    List.of());
                    assertFiniteTrajectory(path, Rotation2d.kPi, robotConfig);
                }
            }
        }
    }

    @Test
    void redAllianceFlipsGuidePositionAndRobotHeadingTogether() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        DriverStationSim.notifyNewData();
        try {
            GuidedPathPlanner.GuidePoint blue =
                    new GuidedPathPlanner.GuidePoint(
                            new Translation2d(2.0, 1.5), Rotation2d.fromDegrees(30.0));
            GuidedPathPlanner.GuidePoint red = AutoCommands.applyGuidePoint(blue);

            assertEquals(FieldConstants.fieldLength - 2.0, red.position().getX(), 1e-6);
            assertEquals(FieldConstants.fieldWidth - 1.5, red.position().getY(), 1e-6);
            assertEquals(-150.0, red.robotHeading().getDegrees(), 1e-6);
        } finally {
            DriverStationSim.resetData();
            DriverStationSim.notifyNewData();
        }
    }

    @Test
    void generatesPathThroughConfiguredBumpCorridor() {
        Rotation2d crossingHeading =
                AutoPoints.Bump.LEFT_DIAGONAL_OUTER
                        .minus(AutoPoints.Bump.LEFT_DIAGONAL_INNER)
                        .getAngle();
        List<GuidedPathPlanner.GuidePoint> guides =
                List.of(
                        new GuidedPathPlanner.GuidePoint(
                                AutoPoints.Bump.LEFT_DIAGONAL_INNER, crossingHeading),
                        new GuidedPathPlanner.GuidePoint(
                                AutoPoints.Bump.LEFT_DIAGONAL_CENTER, crossingHeading),
                        new GuidedPathPlanner.GuidePoint(
                                AutoPoints.Bump.LEFT_DIAGONAL_OUTER, Rotation2d.kPi));
        List<GuidedPathPlanner.Corridor> corridors =
                List.of(
                        new GuidedPathPlanner.Corridor(
                                AutoPoints.Bump.LEFT_DIAGONAL_INNER,
                                AutoPoints.Bump.LEFT_DIAGONAL_OUTER,
                                0.55));

        PathPlannerPath path =
                GuidedPathPlanner.plan(
                        AutoPoints.Bump.LEFT_DIAGONAL_INNER.minus(new Translation2d(0.5, 0.0)),
                        guides,
                        CONSTRAINTS,
                        0.0,
                        corridors);

        assertTrue(path.numPoints() >= 2);
        assertEquals(180.0, path.getGoalEndState().rotation().getDegrees(), 1e-6);
        assertTrue(
                path.getAllPathPoints().stream()
                        .allMatch(
                                point ->
                                        Double.isFinite(point.position.getX())
                                                && Double.isFinite(point.position.getY())
                                                && Double.isFinite(point.distanceAlongPath)
                                                && Double.isFinite(point.maxV)));
    }

    @Test
    void bumpReturnEndsAtInnerPointAlreadyAimedForShooting() {
        for (AutoSelector.Side side : AutoSelector.Side.values()) {
            Translation2d inner =
                    side == AutoSelector.Side.LEFT
                            ? AutoPoints.Bump.LEFT_DIAGONAL_INNER
                            : AutoPoints.Bump.RIGHT_DIAGONAL_INNER;
            Translation2d oldLaunch =
                    side == AutoSelector.Side.LEFT
                            ? AutoPoints.Launch.LEFT_BUMP.getTranslation()
                            : AutoPoints.Launch.RIGHT_BUMP.getTranslation();

            List<GuidedPathPlanner.GuidePoint> guides = AutoCommands.bumpReturnGuides(side);
            GuidedPathPlanner.GuidePoint end = guides.get(guides.size() - 1);
            Rotation2d crossingHeading = inner.minus(guides.get(0).position()).getAngle();

            assertEquals(4, guides.size());
            assertEquals(inner, end.position());
            assertFalse(end.position().equals(oldLaunch));
            for (int i = 0; i < guides.size() - 1; i++) {
                assertEquals(
                        crossingHeading.getRadians(),
                        guides.get(i).robotHeading().getRadians(),
                        1e-9);
            }
            assertEquals(
                    FieldConstants.aimedAtBlueHub(inner)
                            .getRotation()
                            .plus(Rotation2d.kPi)
                            .getRadians(),
                    end.robotHeading().getRadians(),
                    1e-9);
        }
    }

    private static void assertFiniteTrajectory(
            PathPlannerPath path, Rotation2d startingRotation, RobotConfig robotConfig) {
        var trajectory =
                path.generateTrajectory(new ChassisSpeeds(), startingRotation, robotConfig);
        assertTrue(Double.isFinite(trajectory.getTotalTimeSeconds()));
        assertTrue(trajectory.getTotalTimeSeconds() > 0.0);
        assertTrue(
                trajectory.getStates().stream()
                        .allMatch(
                                state ->
                                        Double.isFinite(state.timeSeconds)
                                                && Double.isFinite(state.linearVelocity)
                                                && Double.isFinite(state.pose.getX())
                                                && Double.isFinite(state.pose.getY())
                                                && Double.isFinite(
                                                        state.pose.getRotation().getRadians())));
    }
}
