package lib.ironpulse.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import lombok.Builder;
import lombok.experimental.SuperBuilder;

@SuperBuilder
public class SwerveConfig {
    // general
    public final String name;
    public final double dtS;

    // transmissions
    public final Distance wheelDiameter;
    public final double driveGearRatio;
    public final double steerGearRatio;
    public final double driveMotorKt;

    /** Drive motor winding resistance, ohms; used by the current budget model. */
    public final double driveMotorR;

    public final Mass driveMass;

    /** Chassis yaw inertia about the center of mass. */
    public final MomentOfInertia chassisMoiZ;

    @Builder.Default public final Translation2d comOffset = Translation2d.kZero;

    @Builder.Default
    public final LinearVelocity lowSpeedDeadband =
            edu.wpi.first.units.Units.MetersPerSecond.of(0.01);

    // pigeon
    public final ImuPigeonConfig pigeonConfig;

    // limits
    public final SwerveLimit defaultSwerveLimit;
    public final SwerveModuleLimit defaultSwerveModuleLimit;

    // modules
    public final SwerveModuleConfig[] moduleConfigs;

    public int moduleCount() {
        return moduleConfigs.length;
    }

    public Translation2d[] moduleLocations() {
        Translation2d[] locations = new Translation2d[moduleCount()];
        for (int i = 0; i < moduleCount(); i++) locations[i] = moduleConfigs[i].location;
        return locations;
    }

    /** Reject incomplete force-model data before any drive command can run. */
    public void validateForceModel() {
        if (!positiveFinite(dtS)
                || wheelDiameter == null
                || !positiveFinite(wheelDiameter.in(edu.wpi.first.units.Units.Meters))
                || !positiveFinite(driveGearRatio)
                || !positiveFinite(driveMotorKt)
                || !positiveFinite(driveMotorR)
                || driveMass == null
                || !positiveFinite(driveMass.in(edu.wpi.first.units.Units.Kilograms))
                || chassisMoiZ == null
                || !positiveFinite(chassisMoiZ.in(edu.wpi.first.units.Units.KilogramSquareMeters))
                || comOffset == null
                || !Double.isFinite(comOffset.getX())
                || !Double.isFinite(comOffset.getY())
                || lowSpeedDeadband == null
                || !Double.isFinite(lowSpeedDeadband.in(edu.wpi.first.units.Units.MetersPerSecond))
                || lowSpeedDeadband.in(edu.wpi.first.units.Units.MetersPerSecond) < 0.0) {
            throw new IllegalArgumentException("Invalid or missing swerve force-model parameters");
        }
    }

    private static boolean positiveFinite(double value) {
        return Double.isFinite(value) && value > 0.0;
    }

    @Builder
    public static class SwerveModuleConfig {
        public String name;
        public Translation2d location;
        public int driveMotorId;
        public int steerMotorId;
        public int encoderId;
        public Angle driveMotorEncoderOffset;
        public Angle steerMotorEncoderOffset;
        public boolean driveInverted;
        public boolean steerInverted;
        public boolean encoderInverted;
    }
}
