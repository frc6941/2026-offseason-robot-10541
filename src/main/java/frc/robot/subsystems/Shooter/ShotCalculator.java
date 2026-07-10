package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Single source of truth for distance-based aiming on the drum shooter: maps distance-to-hub to a
 * {@link ShotSolution} (hood angle + flywheel speed) by 1D interpolation, and provides the
 * shoot-on-move {@link #effectiveDistance} lookahead.
 *
 * <p>The interpolation breakpoints, hood angles, flywheel speeds, and times of flight are fixed in
 * {@link ShootingParams}. The interpolation maps are built once at startup.
 *
 * <p>{@link InterpolatingDoubleTreeMap#get} clamps to the nearest endpoint for distances outside
 * the tabulated range, so out-of-range lookups are safe.
 *
 * <p>Shoot-on-move (Step 1): {@link #effectiveDistance} converts the geometric robot→hub distance
 * into the distance the shot must actually cover given the chassis velocity, by a fixed-point
 * lookahead. Feeding that effective distance into {@link #solve} aims hood + flywheel for the
 * moving shot. The chassis heading lead and velocity feedforwards are handled by the aim command
 * (later steps); a linear-drag model on the offset is a deliberate later refinement.
 */
public class ShotCalculator {
    // Fixed interpolation breakpoints (m).
    private static final double[] BREAKPOINTS_M = {2.0, 3.0, 4.0, 5.0, 6.0};

    // Iterations for the shoot-on-move fixed point. Converges in a few; extra are cheap insurance.
    private static final int LOOKAHEAD_ITERATIONS = 10;

    private final InterpolatingDoubleTreeMap hoodAngleDeg = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap shooterRps = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap timeOfFlightSec = new InterpolatingDoubleTreeMap();

    public ShotCalculator() {
        initializeTables();
    }

    /** Resolve a full shot solution for the given distance to the hub (meters). */
    public ShotSolution solve(double distanceMeters) {
        return new ShotSolution(
                Degrees.of(hoodAngleDeg.get(distanceMeters)),
                RotationsPerSecond.of(shooterRps.get(distanceMeters)));
    }

    /** Naive time of flight (s) for a static distance — no velocity compensation. */
    public double timeOfFlightFor(double distanceMeters) {
        return timeOfFlightSec.get(distanceMeters);
    }

    /**
     * Shoot-on-move lookahead: the distance the shot must actually cover given chassis motion.
     *
     * <p>A ball launched while the chassis moves at field velocity {@code (vx, vy)} inherits that
     * velocity for its time of flight, drifting {@code v * tof}. Time of flight itself depends on
     * distance, so this is a fixed point: estimate tof at the current distance, shift the launch
     * point by {@code v * tof}, remeasure distance, repeat. Feed the result into {@link #solve}.
     *
     * <p>When stationary ({@code vx == vy == 0}) this returns the plain geometric distance, so it
     * is always safe to route the normal (non-moving) case through here too.
     *
     * @param shooter field-relative shooter position (≈ robot center for a near-centered shooter)
     * @param target field-relative hub target
     * @param vxField field-relative chassis x velocity (m/s)
     * @param vyField field-relative chassis y velocity (m/s)
     * @return effective distance (m) to feed {@link #solve}
     */
    public double effectiveDistance(
            Translation2d shooter, Translation2d target, double vxField, double vyField) {
        // TODO: add 6328-style linear-drag model to the offset (v*effectiveTOF) once stationary aim
        // is dialed in
        double distance = target.getDistance(shooter);
        for (int i = 0; i < LOOKAHEAD_ITERATIONS; i++) {
            double tof = timeOfFlightSec.get(distance);
            Translation2d virtualShooter =
                    shooter.plus(new Translation2d(vxField * tof, vyField * tof));
            distance = target.getDistance(virtualShooter);
        }
        return distance;
    }

    private void initializeTables() {
        hoodAngleDeg.clear();
        hoodAngleDeg.put(BREAKPOINTS_M[0], ShootingParams.hoodAngleDeg2m);
        hoodAngleDeg.put(BREAKPOINTS_M[1], ShootingParams.hoodAngleDeg3m);
        hoodAngleDeg.put(BREAKPOINTS_M[2], ShootingParams.hoodAngleDeg4m);
        hoodAngleDeg.put(BREAKPOINTS_M[3], ShootingParams.hoodAngleDeg5m);
        hoodAngleDeg.put(BREAKPOINTS_M[4], ShootingParams.hoodAngleDeg6m);

        shooterRps.clear();
        shooterRps.put(BREAKPOINTS_M[0], ShootingParams.shooterRps2m);
        shooterRps.put(BREAKPOINTS_M[1], ShootingParams.shooterRps3m);
        shooterRps.put(BREAKPOINTS_M[2], ShootingParams.shooterRps4m);
        shooterRps.put(BREAKPOINTS_M[3], ShootingParams.shooterRps5m);
        shooterRps.put(BREAKPOINTS_M[4], ShootingParams.shooterRps6m);

        timeOfFlightSec.clear();
        timeOfFlightSec.put(BREAKPOINTS_M[0], ShootingParams.tofSec2m);
        timeOfFlightSec.put(BREAKPOINTS_M[1], ShootingParams.tofSec3m);
        timeOfFlightSec.put(BREAKPOINTS_M[2], ShootingParams.tofSec4m);
        timeOfFlightSec.put(BREAKPOINTS_M[3], ShootingParams.tofSec5m);
        timeOfFlightSec.put(BREAKPOINTS_M[4], ShootingParams.tofSec6m);
    }

    /**
     * Fixed aiming parameters. Values are placeholders — TUNE ON REAL ROBOT. The breakpoint
     * distances themselves are fixed in {@link #BREAKPOINTS_M}; only the hood angle / flywheel
     * speed / time-of-flight at each distance, and the chassis heading tolerance, are exposed here.
     */
    public static final class ShootingParams {
        // TODO: tune hood angle (deg) per distance breakpoint on the real robot
        public static final double hoodAngleDeg2m = 10.0;
        public static final double hoodAngleDeg3m = 18.0;
        public static final double hoodAngleDeg4m = 24.0;
        public static final double hoodAngleDeg5m = 28.0;
        public static final double hoodAngleDeg6m = 30.0;

        // TODO: tune flywheel speed (rotations/sec) per distance breakpoint on the real robot
        public static final double shooterRps2m = 55.0;
        public static final double shooterRps3m = 62.0;
        public static final double shooterRps4m = 70.0;
        public static final double shooterRps5m = 78.0;
        public static final double shooterRps6m = 85.0;
        public static final double lowerShooterSpeedScale = 1.0;

        // TODO: measure ball time of flight (s) per distance breakpoint (drives shoot-on-move lead)
        public static final double tofSec2m = 0.85;
        public static final double tofSec3m = 0.95;
        public static final double tofSec4m = 1.05;
        public static final double tofSec5m = 1.15;
        public static final double tofSec6m = 1.25;

        // TODO: tune chassis heading tolerance (deg) for the "aimed at hub" gate
        public static final double headingToleranceDeg = 2.0;
    }
}
