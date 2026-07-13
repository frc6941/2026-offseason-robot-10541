package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class ShotCalculatorTest {
    private final ShotCalculator calculator = new ShotCalculator();

    @Test
    void returnsExactValuesAtBreakpoints() {
        ShotSolution solution = calculator.solve(4.0);

        assertEquals(24.0, solution.hoodAngle().in(Degrees), 1e-9);
        assertEquals(70.0, solution.shooterSpeed().in(RotationsPerSecond), 1e-9);
        assertEquals(1.05, calculator.timeOfFlightFor(4.0), 1e-9);
    }

    @Test
    void linearlyInterpolatesBetweenBreakpoints() {
        ShotSolution solution = calculator.solve(2.5);

        assertEquals(14.0, solution.hoodAngle().in(Degrees), 1e-9);
        assertEquals(58.5, solution.shooterSpeed().in(RotationsPerSecond), 1e-9);
        assertEquals(0.90, calculator.timeOfFlightFor(2.5), 1e-9);
    }

    @Test
    void clampsDistancesOutsideTableRange() {
        ShotSolution near = calculator.solve(0.5);
        ShotSolution far = calculator.solve(8.0);

        assertEquals(10.0, near.hoodAngle().in(Degrees), 1e-9);
        assertEquals(55.0, near.shooterSpeed().in(RotationsPerSecond), 1e-9);
        assertEquals(30.0, far.hoodAngle().in(Degrees), 1e-9);
        assertEquals(85.0, far.shooterSpeed().in(RotationsPerSecond), 1e-9);
    }

    @Test
    void stationaryLookaheadLeavesDistanceUnchanged() {
        Translation2d shooter = new Translation2d(1.0, 2.0);
        Translation2d target = new Translation2d(5.0, 5.0);

        double effectiveDistance = calculator.effectiveDistance(shooter, target, 0.0, 0.0);

        assertEquals(shooter.getDistance(target), effectiveDistance, 1e-9);
        assertTrue(Double.isFinite(effectiveDistance));
    }
}
