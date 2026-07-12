package frc.robot.subsystems.Shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ShotCalculatorTest {
    private static final double LAUNCH_HEIGHT_METERS = 0.500;
    private static final double TARGET_HEIGHT_METERS = 1.828;
    private static final double APEX_HEIGHT_METERS = 2.328;
    private static final double GRAVITY_METERS_PER_SECOND_SQUARED = 9.80665;

    @Test
    void calculatedTrajectoryPassesThroughApexAndTarget() {
        double distanceMeters = 4.0;
        ShotCalculator.PhysicsSolution solution = calculate(distanceMeters);

        double timeToApex =
                solution.verticalSpeedMetersPerSecond() / GRAVITY_METERS_PER_SECOND_SQUARED;
        double apexX = solution.horizontalSpeedMetersPerSecond() * timeToApex;
        double apexY =
                heightAt(
                        timeToApex,
                        solution.verticalSpeedMetersPerSecond(),
                        GRAVITY_METERS_PER_SECOND_SQUARED);
        double targetY =
                heightAt(
                        solution.timeOfFlightSeconds(),
                        solution.verticalSpeedMetersPerSecond(),
                        GRAVITY_METERS_PER_SECOND_SQUARED);

        assertEquals(solution.apexDistanceMeters(), apexX, 1e-9);
        assertEquals(APEX_HEIGHT_METERS, apexY, 1e-9);
        assertEquals(TARGET_HEIGHT_METERS, targetY, 1e-9);
        assertEquals(
                distanceMeters,
                solution.horizontalSpeedMetersPerSecond() * solution.timeOfFlightSeconds(),
                1e-9);
    }

    @Test
    void solutionsRemainFiniteAcrossExpectedFieldDistances() {
        for (double distanceMeters = 1.0; distanceMeters <= 8.0; distanceMeters += 0.25) {
            ShotCalculator.PhysicsSolution solution = calculate(distanceMeters);
            assertTrue(Double.isFinite(solution.launchAngleRad()));
            assertTrue(solution.launchAngleRad() > 0.0);
            assertTrue(Double.isFinite(solution.launchSpeedMetersPerSecond()));
            assertTrue(solution.launchSpeedMetersPerSecond() > 0.0);
            assertTrue(Double.isFinite(solution.timeOfFlightSeconds()));
            assertTrue(solution.timeOfFlightSeconds() > 0.0);
        }
    }

    @Test
    void quadraticDragCorrectionStillHitsTargetHeight() {
        for (double distanceMeters : new double[] {2.0, 4.0, 6.0}) {
            ShotCalculator.DragSolution drag =
                    ShotCalculator.calculateWithDrag(
                            distanceMeters,
                            LAUNCH_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            APEX_HEIGHT_METERS,
                            GRAVITY_METERS_PER_SECOND_SQUARED,
                            0.215,
                            0.150,
                            0.47,
                            1.225,
                            0.002);
            ShotCalculator.PhysicsSolution ideal = calculate(distanceMeters);

            assertTrue(drag.valid());
            assertEquals(TARGET_HEIGHT_METERS, drag.heightAtTargetMeters(), 1e-6);
            assertTrue(drag.launchSpeedMetersPerSecond() > ideal.launchSpeedMetersPerSecond());
            assertTrue(drag.timeOfFlightSeconds() > 0.0);
        }
    }

    private static ShotCalculator.PhysicsSolution calculate(double distanceMeters) {
        return ShotCalculator.calculateIdeal(
                distanceMeters,
                LAUNCH_HEIGHT_METERS,
                TARGET_HEIGHT_METERS,
                APEX_HEIGHT_METERS,
                GRAVITY_METERS_PER_SECOND_SQUARED);
    }

    private static double heightAt(
            double timeSeconds, double verticalSpeed, double gravityMetersPerSecondSquared) {
        return LAUNCH_HEIGHT_METERS
                + verticalSpeed * timeSeconds
                - 0.5 * gravityMetersPerSecondSquared * timeSeconds * timeSeconds;
    }
}
