package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Single source of truth for distance-based aiming on the drum shooter: maps distance-to-hub to a
 * {@link ShotSolution} (hood angle + flywheel speed) by 1D interpolation.
 *
 * <p>{@link InterpolatingDoubleTreeMap#get} clamps to the nearest endpoint for distances outside the
 * tabulated range, so out-of-range lookups are safe.
 *
 * <p>Shoot-on-move (interpolating additionally on robot velocity / look-ahead, as a turret robot
 * would) is intentionally NOT modeled yet — keep this distance-only until stationary aiming is
 * dialed in, since chassis motion and aim yaw are coupled here.
 *
 * <p>TODO: TUNE ON REAL ROBOT — every table value below is a placeholder.
 */
public class ShotCalculator {
    // Distance (m) -> hood angle (deg)
    private static final InterpolatingDoubleTreeMap HOOD_ANGLE_DEG = new InterpolatingDoubleTreeMap();
    // Distance (m) -> flywheel speed (rotations/sec)
    private static final InterpolatingDoubleTreeMap SHOOTER_RPS = new InterpolatingDoubleTreeMap();

    static {
        // TUNE: distance (m) -> hood angle (deg)
        HOOD_ANGLE_DEG.put(2.0, 10.0);
        HOOD_ANGLE_DEG.put(3.0, 18.0);
        HOOD_ANGLE_DEG.put(4.0, 24.0);
        HOOD_ANGLE_DEG.put(5.0, 28.0);
        HOOD_ANGLE_DEG.put(6.0, 30.0);

        // TUNE: distance (m) -> flywheel speed (rotations/sec)
        SHOOTER_RPS.put(2.0, 55.0);
        SHOOTER_RPS.put(3.0, 62.0);
        SHOOTER_RPS.put(4.0, 70.0);
        SHOOTER_RPS.put(5.0, 78.0);
        SHOOTER_RPS.put(6.0, 85.0);
    }

    /** Resolve a full shot solution for the given distance to the hub (meters). */
    public ShotSolution solve(double distanceMeters) {
        return new ShotSolution(
                Degrees.of(HOOD_ANGLE_DEG.get(distanceMeters)),
                RotationsPerSecond.of(SHOOTER_RPS.get(distanceMeters)));
    }
}
