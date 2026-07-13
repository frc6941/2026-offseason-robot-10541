package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import lib.ntext.NTParameter;

/** Computes shooter setpoints with the reference HTML's quadratic-drag projectile model. */
public class ShotCalculator {
    private static final double LAUNCH_HEIGHT_METERS = 0.500;
    private static final double TARGET_HEIGHT_METERS = 1.828;
    private static final double APEX_HEIGHT_METERS = 2.2;
    private static final double GRAVITY_METERS_PER_SECOND_SQUARED = 9.80665;

    private static final double FUEL_MASS_KG = 0.215;
    private static final double FUEL_DIAMETER_METERS = 0.150;
    private static final double DRAG_COEFFICIENT = 0.47;
    private static final double AIR_DENSITY_KG_PER_CUBIC_METER = 1.225;
    private static final double SIMULATION_DT_SECONDS = 0.002;
    private static final int MAX_SIMULATION_STEPS = 60_000;
    private static final int DRAG_BINARY_SEARCH_ITERATIONS = 46;
    private static final int DRAG_UPPER_BOUND_EXPANSIONS = 20;
    private static final double MAX_DRAG_SEARCH_SPEED_MPS = 120.0;

    private static final double SHOOTER_WHEEL_DIAMETER_METERS = 0.100;
    private static final double NOMINAL_EXIT_EFFICIENCY = 0.75;
    private static final double MAX_SHOOTER_RPS = 85.0;

    // Fitted from the old hood table. These describe mechanism geometry, not shot tuning.
    private static final double LAUNCH_ANGLE_AT_HOOD_ZERO_DEG = 85.12;
    private static final double LAUNCH_ANGLE_CHANGE_PER_HOOD_DEGREE = -1.349;
    // Real-shot calibration: the previous fitted hood mapping produced an apex near 3 m. Moving
    // the mechanism +6 deg lowers the physical launch angle by about 8.1 deg.
    private static final double HOOD_CALIBRATION_OFFSET_DEG = 6.0;

    private static final double LOWER_SHOOTER_SPEED_SCALE = 1.0;
    private static final double HEADING_TOLERANCE_DEG = 2.0;
    private static final double MIN_DISTANCE_METERS = 0.05;
    private static final double POWER_SCALE_REFERENCE_DISTANCE_METERS = 4.0;
    private static final int DISTANCE_CACHE_BINS_PER_METER = 100;
    private static final int LOOKAHEAD_ITERATIONS = 10;

    // The expensive drag solve is independent of the live calibration values. Cache it at 1 cm
    // distance resolution so periodic aiming remains comfortably inside the robot loop budget.
    private final Map<Integer, DragSolution> dragSolutionCache = new HashMap<>();

    public ShotSolution solve(double distanceMeters) {
        DragSolution drag = dragSolutionFor(distanceMeters);

        double rawHoodAngleDeg =
                (Math.toDegrees(drag.launchAngleRad()) - LAUNCH_ANGLE_AT_HOOD_ZERO_DEG)
                                / LAUNCH_ANGLE_CHANGE_PER_HOOD_DEGREE
                        + HOOD_CALIBRATION_OFFSET_DEG
                        + ShooterTuningParamsNT.hoodTrimDeg.getValue();
        double hoodAngleDeg =
                MathUtil.clamp(
                        rawHoodAngleDeg,
                        ShooterConfig.HOOD_MIN_ANGLE.in(Degrees),
                        ShooterConfig.HOOD_MAX_ANGLE.in(Degrees));

        double rawShooterRps =
                drag.launchSpeedMetersPerSecond()
                        / (Math.PI * SHOOTER_WHEEL_DIAMETER_METERS * NOMINAL_EXIT_EFFICIENCY)
                        * shotPowerScaleFor(distanceMeters);
        double shooterRps = MathUtil.clamp(rawShooterRps, 0.0, MAX_SHOOTER_RPS);

        return new ShotSolution(
                Degrees.of(hoodAngleDeg),
                RotationsPerSecond.of(shooterRps),
                Degrees.of(Math.toDegrees(drag.launchAngleRad())),
                MetersPerSecond.of(drag.launchSpeedMetersPerSecond()),
                Seconds.of(drag.timeOfFlightSeconds()),
                Degrees.of(rawHoodAngleDeg),
                RotationsPerSecond.of(rawShooterRps),
                Meters.of(drag.maxHeightMeters()),
                drag.valid());
    }

    public double timeOfFlightFor(double distanceMeters) {
        return dragSolutionFor(distanceMeters).timeOfFlightSeconds();
    }

    public double effectiveDistance(
            Translation2d shooter, Translation2d target, double vxField, double vyField) {
        double distance = target.getDistance(shooter);
        for (int i = 0; i < LOOKAHEAD_ITERATIONS; i++) {
            double tof = timeOfFlightFor(distance);
            Translation2d virtualShooter =
                    shooter.plus(new Translation2d(vxField * tof, vyField * tof));
            distance = target.getDistance(virtualShooter);
        }
        return distance;
    }

    public double lowerShooterSpeedScale() {
        return LOWER_SHOOTER_SPEED_SCALE;
    }

    public double headingToleranceDeg() {
        return HEADING_TOLERANCE_DEG;
    }

    public double shotPowerScaleFor(double distanceMeters) {
        return calculatePowerScale(
                distanceMeters,
                ShooterTuningParamsNT.shotPowerScale.getValue(),
                ShooterTuningParamsNT.shotPowerSlopePerMeter.getValue());
    }

    static double calculatePowerScale(
            double distanceMeters, double scaleAtReference, double slopePerMeter) {
        double distance = Math.max(MIN_DISTANCE_METERS, Math.abs(distanceMeters));
        return Math.max(
                0.0,
                scaleAtReference
                        + slopePerMeter * (distance - POWER_SCALE_REFERENCE_DISTANCE_METERS));
    }

    /** Samples a 3D quadratic-drag trajectory for AdvantageScope visualization. */
    public Pose3d[] sampleDragPath(
            Pose3d releasePose,
            Rotation2d yaw,
            ShotSolution solution,
            Translation2d inheritedFieldVelocity) {
        double pitch = solution.launchAngle().in(Degrees) * Math.PI / 180.0;
        double speed = solution.launchSpeed().in(MetersPerSecond);
        double horizontalSpeed = speed * Math.cos(pitch);
        double vx = horizontalSpeed * yaw.getCos() + inheritedFieldVelocity.getX();
        double vy = horizontalSpeed * yaw.getSin() + inheritedFieldVelocity.getY();
        double vz = speed * Math.sin(pitch);
        Translation3d position = releasePose.getTranslation();
        double radius = FUEL_DIAMETER_METERS * 0.5;
        double area = Math.PI * radius * radius;
        double dragAccelerationFactor =
                0.5 * AIR_DENSITY_KG_PER_CUBIC_METER * DRAG_COEFFICIENT * area / FUEL_MASS_KG;
        int sampleEverySteps = Math.max(1, (int) Math.round(0.02 / SIMULATION_DT_SECONDS));
        int maxSteps = (int) Math.ceil(3.0 / SIMULATION_DT_SECONDS);
        List<Pose3d> samples = new ArrayList<>();
        samples.add(new Pose3d(position, new Rotation3d()));

        for (int i = 0; i < maxSteps; i++) {
            double airSpeed = Math.sqrt(vx * vx + vy * vy + vz * vz);
            double ax = -dragAccelerationFactor * airSpeed * vx;
            double ay = -dragAccelerationFactor * airSpeed * vy;
            double az = -GRAVITY_METERS_PER_SECOND_SQUARED - dragAccelerationFactor * airSpeed * vz;
            vx += ax * SIMULATION_DT_SECONDS;
            vy += ay * SIMULATION_DT_SECONDS;
            vz += az * SIMULATION_DT_SECONDS;
            position = position.plus(new Translation3d(vx, vy, vz).times(SIMULATION_DT_SECONDS));
            if ((i + 1) % sampleEverySteps == 0) {
                samples.add(new Pose3d(position, new Rotation3d()));
            }
            if (position.getZ() <= 0.0) {
                break;
            }
        }
        return samples.toArray(Pose3d[]::new);
    }

    private DragSolution dragSolutionFor(double distanceMeters) {
        double distance = Math.max(MIN_DISTANCE_METERS, Math.abs(distanceMeters));
        int cacheKey = (int) Math.round(distance * DISTANCE_CACHE_BINS_PER_METER);
        return dragSolutionCache.computeIfAbsent(
                cacheKey,
                key ->
                        calculateWithDrag(
                                key / (double) DISTANCE_CACHE_BINS_PER_METER,
                                LAUNCH_HEIGHT_METERS,
                                TARGET_HEIGHT_METERS,
                                APEX_HEIGHT_METERS,
                                GRAVITY_METERS_PER_SECOND_SQUARED,
                                FUEL_MASS_KG,
                                FUEL_DIAMETER_METERS,
                                DRAG_COEFFICIENT,
                                AIR_DENSITY_KG_PER_CUBIC_METER,
                                SIMULATION_DT_SECONDS));
    }

    static DragSolution calculateWithDrag(
            double distanceMeters,
            double launchHeightMeters,
            double targetHeightMeters,
            double apexHeightMeters,
            double gravityMetersPerSecondSquared,
            double fuelMassKg,
            double fuelDiameterMeters,
            double dragCoefficient,
            double airDensityKgPerCubicMeter,
            double dtSeconds) {
        PhysicsSolution ideal =
                calculateIdeal(
                        distanceMeters,
                        launchHeightMeters,
                        targetHeightMeters,
                        apexHeightMeters,
                        gravityMetersPerSecondSquared);

        double lowerSpeed = 0.05;
        double upperSpeed = Math.max(4.0, ideal.launchSpeedMetersPerSecond() * 1.2);
        DragSimulation upper =
                simulateDrag(
                        distanceMeters,
                        launchHeightMeters,
                        ideal.launchAngleRad(),
                        upperSpeed,
                        gravityMetersPerSecondSquared,
                        fuelMassKg,
                        fuelDiameterMeters,
                        dragCoefficient,
                        airDensityKgPerCubicMeter,
                        dtSeconds);
        int expansions = 0;
        while ((!upper.valid() || upper.heightAtTargetMeters() < targetHeightMeters)
                && upperSpeed < MAX_DRAG_SEARCH_SPEED_MPS
                && expansions < DRAG_UPPER_BOUND_EXPANSIONS) {
            upperSpeed *= 1.35;
            upper =
                    simulateDrag(
                            distanceMeters,
                            launchHeightMeters,
                            ideal.launchAngleRad(),
                            upperSpeed,
                            gravityMetersPerSecondSquared,
                            fuelMassKg,
                            fuelDiameterMeters,
                            dragCoefficient,
                            airDensityKgPerCubicMeter,
                            dtSeconds);
            expansions++;
        }

        if (!upper.valid() || upper.heightAtTargetMeters() < targetHeightMeters) {
            return new DragSolution(
                    ideal.launchAngleRad(),
                    ideal.launchSpeedMetersPerSecond(),
                    ideal.timeOfFlightSeconds(),
                    ideal.apexHeightMeters(),
                    ideal.apexDistanceMeters(),
                    targetHeightMeters,
                    false);
        }

        for (int i = 0; i < DRAG_BINARY_SEARCH_ITERATIONS; i++) {
            double speed = (lowerSpeed + upperSpeed) * 0.5;
            DragSimulation simulation =
                    simulateDrag(
                            distanceMeters,
                            launchHeightMeters,
                            ideal.launchAngleRad(),
                            speed,
                            gravityMetersPerSecondSquared,
                            fuelMassKg,
                            fuelDiameterMeters,
                            dragCoefficient,
                            airDensityKgPerCubicMeter,
                            dtSeconds);
            if (!simulation.valid() || simulation.heightAtTargetMeters() < targetHeightMeters) {
                lowerSpeed = speed;
            } else {
                upperSpeed = speed;
            }
        }

        DragSimulation corrected =
                simulateDrag(
                        distanceMeters,
                        launchHeightMeters,
                        ideal.launchAngleRad(),
                        upperSpeed,
                        gravityMetersPerSecondSquared,
                        fuelMassKg,
                        fuelDiameterMeters,
                        dragCoefficient,
                        airDensityKgPerCubicMeter,
                        dtSeconds);
        return new DragSolution(
                ideal.launchAngleRad(),
                upperSpeed,
                corrected.timeAtTargetSeconds(),
                corrected.maxHeightMeters(),
                corrected.maxHeightDistanceMeters(),
                corrected.heightAtTargetMeters(),
                corrected.valid());
    }

    static PhysicsSolution calculateIdeal(
            double distanceMeters,
            double launchHeightMeters,
            double targetHeightMeters,
            double apexHeightMeters,
            double gravityMetersPerSecondSquared) {
        double distance = Math.max(MIN_DISTANCE_METERS, Math.abs(distanceMeters));
        double apex =
                Math.max(apexHeightMeters, Math.max(launchHeightMeters, targetHeightMeters) + 1e-4);
        double launchToApex = apex - launchHeightMeters;
        double targetToApex = apex - targetHeightMeters;
        double sqrtLaunch = Math.sqrt(launchToApex);
        double sqrtTarget = Math.sqrt(targetToApex);
        double apexDistance = distance * sqrtLaunch / (sqrtLaunch + sqrtTarget);
        double parabolaCoefficient = launchToApex / (apexDistance * apexDistance);
        double launchAngleRad = Math.atan2(2.0 * launchToApex, apexDistance);
        double horizontalSpeed =
                Math.sqrt(gravityMetersPerSecondSquared / (2.0 * parabolaCoefficient));
        double verticalSpeed = horizontalSpeed * Math.tan(launchAngleRad);
        double launchSpeed = Math.hypot(horizontalSpeed, verticalSpeed);
        double timeOfFlight = distance / horizontalSpeed;
        return new PhysicsSolution(
                launchAngleRad,
                launchSpeed,
                horizontalSpeed,
                verticalSpeed,
                timeOfFlight,
                apexDistance,
                apex);
    }

    private static DragSimulation simulateDrag(
            double distanceMeters,
            double launchHeightMeters,
            double launchAngleRad,
            double launchSpeedMetersPerSecond,
            double gravityMetersPerSecondSquared,
            double fuelMassKg,
            double fuelDiameterMeters,
            double dragCoefficient,
            double airDensityKgPerCubicMeter,
            double dtSeconds) {
        double distance = Math.max(MIN_DISTANCE_METERS, Math.abs(distanceMeters));
        double dt = Math.max(0.0005, dtSeconds);
        double mass = Math.max(1e-4, fuelMassKg);
        double radius = Math.max(1e-4, fuelDiameterMeters * 0.5);
        double area = Math.PI * radius * radius;
        double dragAccelerationFactor =
                0.5 * airDensityKgPerCubicMeter * dragCoefficient * area / mass;

        double x = 0.0;
        double y = launchHeightMeters;
        double vx = launchSpeedMetersPerSecond * Math.cos(launchAngleRad);
        double vy = launchSpeedMetersPerSecond * Math.sin(launchAngleRad);
        double time = 0.0;
        double maxHeight = y;
        double maxHeightDistance = x;

        for (int i = 0; i < MAX_SIMULATION_STEPS; i++) {
            double previousX = x;
            double previousY = y;
            double previousTime = time;
            double speed = Math.hypot(vx, vy);
            double ax = speed < 1e-9 ? 0.0 : -dragAccelerationFactor * speed * vx;
            double ay =
                    -gravityMetersPerSecondSquared
                            + (speed < 1e-9 ? 0.0 : -dragAccelerationFactor * speed * vy);

            // Same semi-implicit Euler integration used by the reference HTML.
            vx += ax * dt;
            vy += ay * dt;
            x += vx * dt;
            y += vy * dt;
            time += dt;

            if (y > maxHeight) {
                maxHeight = y;
                maxHeightDistance = x;
            }
            if (x >= distance) {
                double denominator = x - previousX;
                double fraction =
                        Math.abs(denominator) < 1e-9 ? 1.0 : (distance - previousX) / denominator;
                return new DragSimulation(
                        true,
                        previousY + fraction * (y - previousY),
                        previousTime + fraction * (time - previousTime),
                        maxHeight,
                        maxHeightDistance);
            }
            if (y < -2.0 || x > distance + 5.0 || x < -1.0) {
                break;
            }
        }
        return new DragSimulation(false, Double.NaN, Double.NaN, maxHeight, maxHeightDistance);
    }

    record PhysicsSolution(
            double launchAngleRad,
            double launchSpeedMetersPerSecond,
            double horizontalSpeedMetersPerSecond,
            double verticalSpeedMetersPerSecond,
            double timeOfFlightSeconds,
            double apexDistanceMeters,
            double apexHeightMeters) {}

    record DragSolution(
            double launchAngleRad,
            double launchSpeedMetersPerSecond,
            double timeOfFlightSeconds,
            double maxHeightMeters,
            double maxHeightDistanceMeters,
            double heightAtTargetMeters,
            boolean valid) {}

    private record DragSimulation(
            boolean valid,
            double heightAtTargetMeters,
            double timeAtTargetSeconds,
            double maxHeightMeters,
            double maxHeightDistanceMeters) {}

    @NTParameter(tableName = "Params/ShooterTuning")
    public static final class ShooterTuningParams {
        // Power scale at 4 m. Adjust this first until a 4 m shot is correct.
        public static final double shotPowerScale = 1.81;

        // Distance correction around 4 m. More negative adds near power and removes far power.
        public static final double shotPowerSlopePerMeter = -0.01;

        // Direct hood mechanism trim. Positive moves the hood toward its positive-angle limit.
        public static final double hoodTrimDeg = 0.0;

        private ShooterTuningParams() {}
    }
}
