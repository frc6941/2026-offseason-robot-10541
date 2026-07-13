package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import lib.ntext.NTParameter;

/** Resolves hood angle, shooter speed, and time of flight from live interpolation tables. */
public class ShotCalculator {
    private static final double[] BREAKPOINTS_METERS = {2.0, 3.0, 4.0, 5.0, 6.0};
    private static final int LOOKAHEAD_ITERATIONS = 10;

    private final InterpolatingDoubleTreeMap hoodAngleDeg = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap shooterRps = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap timeOfFlightSec = new InterpolatingDoubleTreeMap();

    public ShotSolution solve(double distanceMeters) {
        rebuildTables();
        return new ShotSolution(
                Degrees.of(hoodAngleDeg.get(distanceMeters)),
                RotationsPerSecond.of(shooterRps.get(distanceMeters)));
    }

    public double timeOfFlightFor(double distanceMeters) {
        rebuildTables();
        return timeOfFlightSec.get(distanceMeters);
    }

    /**
     * Compensates distance for the field-relative velocity inherited by a ball launched while the
     * robot is moving. The tabulated time of flight is iterated because it changes with distance.
     */
    public double effectiveDistance(
            Translation2d shooter, Translation2d target, double vxField, double vyField) {
        rebuildTables();
        double distance = target.getDistance(shooter);
        for (int i = 0; i < LOOKAHEAD_ITERATIONS; i++) {
            double timeOfFlight = timeOfFlightSec.get(distance);
            Translation2d virtualShooter =
                    shooter.plus(new Translation2d(vxField * timeOfFlight, vyField * timeOfFlight));
            distance = target.getDistance(virtualShooter);
        }
        return distance;
    }

    public double lowerShooterSpeedScale() {
        return ShootingParamsNT.lowerShooterSpeedScale.getValue();
    }

    public double headingToleranceDeg() {
        return ShootingParamsNT.headingToleranceDeg.getValue();
    }

    private void rebuildTables() {
        hoodAngleDeg.clear();
        hoodAngleDeg.put(BREAKPOINTS_METERS[0], ShootingParamsNT.hoodAngleDeg2m.getValue()); // 23
        hoodAngleDeg.put(BREAKPOINTS_METERS[1], ShootingParamsNT.hoodAngleDeg3m.getValue()); // 29.5
        hoodAngleDeg.put(BREAKPOINTS_METERS[2], ShootingParamsNT.hoodAngleDeg4m.getValue()); // 37
        hoodAngleDeg.put(BREAKPOINTS_METERS[3], ShootingParamsNT.hoodAngleDeg5m.getValue());
        hoodAngleDeg.put(BREAKPOINTS_METERS[4], ShootingParamsNT.hoodAngleDeg6m.getValue());

        shooterRps.clear();
        shooterRps.put(BREAKPOINTS_METERS[0], ShootingParamsNT.shooterRps2m.getValue()); // 55
        shooterRps.put(BREAKPOINTS_METERS[1], ShootingParamsNT.shooterRps3m.getValue()); // 57.5
        shooterRps.put(BREAKPOINTS_METERS[2], ShootingParamsNT.shooterRps4m.getValue()); // 59
        shooterRps.put(BREAKPOINTS_METERS[3], ShootingParamsNT.shooterRps5m.getValue());
        shooterRps.put(BREAKPOINTS_METERS[4], ShootingParamsNT.shooterRps6m.getValue());

        timeOfFlightSec.clear();
        timeOfFlightSec.put(BREAKPOINTS_METERS[0], ShootingParamsNT.tofSec2m.getValue()); // 1.1
        timeOfFlightSec.put(BREAKPOINTS_METERS[1], ShootingParamsNT.tofSec3m.getValue());
        timeOfFlightSec.put(BREAKPOINTS_METERS[2], ShootingParamsNT.tofSec4m.getValue());
        timeOfFlightSec.put(BREAKPOINTS_METERS[3], ShootingParamsNT.tofSec5m.getValue());
        timeOfFlightSec.put(BREAKPOINTS_METERS[4], ShootingParamsNT.tofSec6m.getValue());
    }

    /** Values between breakpoints are linearly interpolated and remain live-tunable over NT. */
    @NTParameter(tableName = "Params/Shooting")
    public static final class ShootingParams {
        public static final double hoodAngleDeg2m = 23.0;
        public static final double hoodAngleDeg3m = 29.5;
        public static final double hoodAngleDeg4m = 37.;
        public static final double hoodAngleDeg5m = 40.0;
        public static final double hoodAngleDeg6m = 37.0;

        public static final double shooterRps2m = 55.0;
        public static final double shooterRps3m = 57.5;
        public static final double shooterRps4m = 59.0;
        public static final double shooterRps5m = 67.0;
        public static final double shooterRps6m = 60.0;
        public static final double lowerShooterSpeedScale = 1.0;

        public static final double tofSec2m = 0.85;
        public static final double tofSec3m = 0.95;
        public static final double tofSec4m = 1.05;
        public static final double tofSec5m = 1.15;
        public static final double tofSec6m = 1.25;

        public static final double headingToleranceDeg = 2.0;

        private ShootingParams() {}
    }
}
