package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import lib.ntext.NTParameter;

/**
 * Single source of truth for distance-based aiming on the drum shooter: maps distance-to-hub to a
 * {@link ShotSolution} (hood angle + flywheel speed) by 1D interpolation.
 *
 * <p>The interpolation breakpoints (distances, in meters) are fixed; the hood angle and flywheel
 * speed at each breakpoint are live-tunable over NetworkTables via {@link ShootingParams}. The maps
 * are rebuilt only when a parameter changes.
 *
 * <p>{@link InterpolatingDoubleTreeMap#get} clamps to the nearest endpoint for distances outside the
 * tabulated range, so out-of-range lookups are safe.
 *
 * <p>Shoot-on-move (interpolating additionally on robot velocity / look-ahead, as a turret robot
 * would) is intentionally NOT modeled yet — keep this distance-only until stationary aiming is
 * dialed in, since chassis motion and aim yaw are coupled here.
 */
public class ShotCalculator {
    // Fixed interpolation breakpoints (m). Only the values at these distances are NT-tunable.
    private static final double[] BREAKPOINTS_M = {2.0, 3.0, 4.0, 5.0, 6.0};

    private final InterpolatingDoubleTreeMap hoodAngleDeg = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap shooterRps = new InterpolatingDoubleTreeMap();
    private boolean built = false;

    /** Resolve a full shot solution for the given distance to the hub (meters). */
    public ShotSolution solve(double distanceMeters) {
        rebuildIfNeeded();
        return new ShotSolution(
                Degrees.of(hoodAngleDeg.get(distanceMeters)),
                RotationsPerSecond.of(shooterRps.get(distanceMeters)));
    }

    private void rebuildIfNeeded() {
        if (built && !ShootingParamsNT.isAnyChanged()) {
            return;
        }
        hoodAngleDeg.clear();
        hoodAngleDeg.put(BREAKPOINTS_M[0], ShootingParamsNT.hoodAngleDeg2m.getValue());
        hoodAngleDeg.put(BREAKPOINTS_M[1], ShootingParamsNT.hoodAngleDeg3m.getValue());
        hoodAngleDeg.put(BREAKPOINTS_M[2], ShootingParamsNT.hoodAngleDeg4m.getValue());
        hoodAngleDeg.put(BREAKPOINTS_M[3], ShootingParamsNT.hoodAngleDeg5m.getValue());
        hoodAngleDeg.put(BREAKPOINTS_M[4], ShootingParamsNT.hoodAngleDeg6m.getValue());

        shooterRps.clear();
        shooterRps.put(BREAKPOINTS_M[0], ShootingParamsNT.shooterRps2m.getValue());
        shooterRps.put(BREAKPOINTS_M[1], ShootingParamsNT.shooterRps3m.getValue());
        shooterRps.put(BREAKPOINTS_M[2], ShootingParamsNT.shooterRps4m.getValue());
        shooterRps.put(BREAKPOINTS_M[3], ShootingParamsNT.shooterRps5m.getValue());
        shooterRps.put(BREAKPOINTS_M[4], ShootingParamsNT.shooterRps6m.getValue());

        built = true;
    }

    /**
     * Live-tunable aiming parameters. Values are placeholders — TUNE ON REAL ROBOT. The breakpoint
     * distances themselves are fixed in {@link #BREAKPOINTS_M}; only the hood angle / flywheel speed
     * at each distance, and the chassis heading tolerance, are exposed here.
     */
    @NTParameter(tableName = "Params/Shooting")
    public static final class ShootingParams {
        // Hood angle (deg) at each distance breakpoint
        public static final double hoodAngleDeg2m = 10.0;
        public static final double hoodAngleDeg3m = 18.0;
        public static final double hoodAngleDeg4m = 24.0;
        public static final double hoodAngleDeg5m = 28.0;
        public static final double hoodAngleDeg6m = 30.0;

        // Flywheel speed (rotations/sec) at each distance breakpoint
        public static final double shooterRps2m = 55.0;
        public static final double shooterRps3m = 62.0;
        public static final double shooterRps4m = 70.0;
        public static final double shooterRps5m = 78.0;
        public static final double shooterRps6m = 85.0;

        // Chassis heading error allowed before we consider ourselves aimed at the hub (deg)
        public static final double headingToleranceDeg = 2.0;
    }
}
