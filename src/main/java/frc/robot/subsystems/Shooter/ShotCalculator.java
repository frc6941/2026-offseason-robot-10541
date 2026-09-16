package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import lib.ntext.NTParameter;

/** Resolves hood angle, shooter speed, and time of flight from live interpolation tables. */
public class ShotCalculator {
    private static final double[] BREAKPOINTS_METERS = {
        1.0, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 6.0, 7.0, 8.0
    };

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

    public double lowerShooterSpeedScale() {
        return ShootingParamsNT.lowerShooterSpeedScale.getValue();
    }

    public double headingToleranceDeg() {
        return ShootingParamsNT.headingToleranceDeg.getValue();
    }

    private void rebuildTables() {
        hoodAngleDeg.clear();
        hoodAngleDeg.put(BREAKPOINTS_METERS[0], ShootingParamsNT.hoodAngleDeg1m.getValue());
        hoodAngleDeg.put(BREAKPOINTS_METERS[1], ShootingParamsNT.hoodAngleDeg2m.getValue()); // 23
        hoodAngleDeg.put(BREAKPOINTS_METERS[2], ShootingParamsNT.hoodAngleDeg3m.getValue()); // 29.5
        hoodAngleDeg.put(BREAKPOINTS_METERS[3], ShootingParamsNT.hoodAngleDeg4m.getValue()); // 37
        hoodAngleDeg.put(BREAKPOINTS_METERS[4], ShootingParamsNT.hoodAngleDeg5m.getValue());
        hoodAngleDeg.put(BREAKPOINTS_METERS[5], ShootingParamsNT.hoodAngleDeg6m.getValue());
        hoodAngleDeg.put(BREAKPOINTS_METERS[6], ShootingParamsNT.hoodAngleDeg7m.getValue());
        hoodAngleDeg.put(BREAKPOINTS_METERS[7], ShootingParamsNT.hoodAngleDeg8m.getValue());
        hoodAngleDeg.put(BREAKPOINTS_METERS[8], ShootingParamsNT.hoodAngleDeg9m.getValue());
        hoodAngleDeg.put(BREAKPOINTS_METERS[9], ShootingParamsNT.hoodAngleDeg10m.getValue());
        hoodAngleDeg.put(BREAKPOINTS_METERS[10], ShootingParamsNT.hoodAngleDeg11m.getValue());

        shooterRps.clear();
        shooterRps.put(BREAKPOINTS_METERS[0], ShootingParamsNT.shooterRps1m.getValue());
        shooterRps.put(BREAKPOINTS_METERS[1], ShootingParamsNT.shooterRps2m.getValue()); // 55
        shooterRps.put(BREAKPOINTS_METERS[2], ShootingParamsNT.shooterRps3m.getValue()); // 57.5
        shooterRps.put(BREAKPOINTS_METERS[3], ShootingParamsNT.shooterRps4m.getValue()); // 59
        shooterRps.put(BREAKPOINTS_METERS[4], ShootingParamsNT.shooterRps5m.getValue());
        shooterRps.put(BREAKPOINTS_METERS[5], ShootingParamsNT.shooterRps6m.getValue());
        shooterRps.put(BREAKPOINTS_METERS[6], ShootingParamsNT.shooterRps7m.getValue());
        shooterRps.put(BREAKPOINTS_METERS[7], ShootingParamsNT.shooterRps8m.getValue());
        shooterRps.put(BREAKPOINTS_METERS[8], ShootingParamsNT.shooterRps9m.getValue());
        shooterRps.put(BREAKPOINTS_METERS[9], ShootingParamsNT.shooterRps10m.getValue());
        shooterRps.put(BREAKPOINTS_METERS[10], ShootingParamsNT.shooterRps11m.getValue());

        timeOfFlightSec.clear();
        timeOfFlightSec.put(BREAKPOINTS_METERS[0], ShootingParamsNT.tofSec1m.getValue());
        timeOfFlightSec.put(BREAKPOINTS_METERS[1], ShootingParamsNT.tofSec2m.getValue()); // 1.1
        timeOfFlightSec.put(BREAKPOINTS_METERS[2], ShootingParamsNT.tofSec3m.getValue());
        timeOfFlightSec.put(BREAKPOINTS_METERS[3], ShootingParamsNT.tofSec4m.getValue());
        timeOfFlightSec.put(BREAKPOINTS_METERS[4], ShootingParamsNT.tofSec5m.getValue());
        timeOfFlightSec.put(BREAKPOINTS_METERS[5], ShootingParamsNT.tofSec6m.getValue());
        timeOfFlightSec.put(BREAKPOINTS_METERS[6], ShootingParamsNT.tofSec7m.getValue());
        timeOfFlightSec.put(BREAKPOINTS_METERS[7], ShootingParamsNT.tofSec8m.getValue());
        timeOfFlightSec.put(BREAKPOINTS_METERS[8], ShootingParamsNT.tofSec9m.getValue());
        timeOfFlightSec.put(BREAKPOINTS_METERS[9], ShootingParamsNT.tofSec10m.getValue());
        timeOfFlightSec.put(BREAKPOINTS_METERS[10], ShootingParamsNT.tofSec11m.getValue());
    }

    /** Values between breakpoints are linearly interpolated and remain live-tunable over NT. */
    @NTParameter(tableName = "Params/Shooting")
    public static final class ShootingParams {
        public static final double hoodAngleDeg1m = 2.0;
        public static final double hoodAngleDeg2m = 14.0;
        public static final double hoodAngleDeg3m = 20.5; // 20.5
        public static final double hoodAngleDeg4m = 20.5; // 20.5
        public static final double hoodAngleDeg5m = 21.5; // 21.5
        public static final double hoodAngleDeg6m = 24.0; // 24
        public static final double hoodAngleDeg7m = 27.0; // 27
        public static final double hoodAngleDeg8m = 30.0;
        public static final double hoodAngleDeg9m = 32.0;
        public static final double hoodAngleDeg10m = 34.0;
        public static final double hoodAngleDeg11m = 36.0;

        public static final double shooterRps1m = 48.0;
        public static final double shooterRps2m = 49.0; // 50
        public static final double shooterRps3m = 50.0; // 51
        public static final double shooterRps4m = 53.0; // 54
        public static final double shooterRps5m = 56.2; // 57.5
        public static final double shooterRps6m = 60.2; // 61.2
        public static final double shooterRps7m = 64.0; // 65
        public static final double shooterRps8m = 68.0;
        public static final double shooterRps9m = 71.0;
        public static final double shooterRps10m = 74.0;
        public static final double shooterRps11m = 77.0;
        public static final double lowerShooterSpeedScale = 1.0;

        public static final double tofSec1m = 0.60;
        public static final double tofSec2m = 0.85;
        public static final double tofSec3m = 0.95;
        public static final double tofSec4m = 1.05;
        public static final double tofSec5m = 1.15;
        public static final double tofSec6m = 1.25;
        public static final double tofSec7m = 1.30;
        public static final double tofSec8m = 1.40;
        public static final double tofSec9m = 1.50;
        public static final double tofSec10m = 1.60;
        public static final double tofSec11m = 1.70;

        public static final double headingToleranceDeg = 2.0;

        private ShootingParams() {}
    }
}
