package lib.ironpulse.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;

public interface ImuIO {
    // read
    default void updateInputs(ImuIOInputs inputs) {}

    default void reset() {}

    void setYawDeg(double yaw);

    double getYawDeg();

    /**
     * Turns a raw accelerometer sample into the robot frame and writes both into {@code inputs}.
     *
     * <p>The mount pose the gyro is configured with rotates yaw, pitch and roll; on Phoenix 6
     * 26.50.0-alpha-1 it does <b>not</b> rotate {@code getAccelerationX/Y}, which come out in the
     * sensor's own yaw frame. Fitting the 2026-08-26 12-34-21 log against the wheels' acceleration
     * gives {@code a_robot = R(−88.5°)·a_raw} against a configured mount yaw of 88.95° — the same
     * rotation, so the correction here is {@code R(−accelFrameYaw)} and nothing is calibrated.
     *
     * <p>{@code accelFrameYaw} is a separate knob from the mount pose precisely so this cannot
     * double-rotate silently if a later Phoenix starts applying the mount pose to the accelerometer
     * too: set it to 0 and this becomes the identity. A physical re-mount at mount pose 0 wants the
     * same 0 and no other change. The raw pair is kept in the inputs either way, so the frame can
     * be re-fitted from any bag after an update.
     *
     * <p>Gravity is left in: removing it needs pitch and roll, which is {@link
     * BodyVelocityEstimator}'s job.
     *
     * @param axRaw sensor-frame acceleration along the sensor's x, m/s²
     * @param ayRaw sensor-frame acceleration along the sensor's y, m/s²
     * @param accelFrameYawDeg the yaw the raw pair is turned back by, degrees
     */
    static void rotateAccelToRobot(
            ImuIOInputs inputs, double axRaw, double ayRaw, double accelFrameYawDeg) {
        double c = Math.cos(Math.toRadians(accelFrameYawDeg));
        double s = Math.sin(Math.toRadians(accelFrameYawDeg));
        inputs.accelRawXMps2 = axRaw;
        inputs.accelRawYMps2 = ayRaw;
        inputs.accelXMps2 = c * axRaw + s * ayRaw;
        inputs.accelYMps2 = -s * axRaw + c * ayRaw;
    }

    @AutoLog
    class ImuIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double yawVelocityRadPerSec = 0.0;
        public double yawVelocityRadPerSecCmd = 0.0;
        public Rotation2d pitchPosition = new Rotation2d();
        public double pitchVelocityRadPerSec = 0.0;
        public Rotation2d rollPosition = new Rotation2d();
        public double rollVelocityRadPerSec = 0.0;

        /** Body-frame linear acceleration, m/s², by {@link #rotateAccelToRobot}; gravity in it. */
        public double accelXMps2 = 0.0;

        public double accelYMps2 = 0.0;

        /** The same acceleration as the sensor reported it, m/s² — the frame's audit trail. */
        public double accelRawXMps2 = 0.0;

        public double accelRawYMps2 = 0.0;

        /**
         * Age of the acceleration sample when it was read, seconds — the transport age Phoenix
         * reports, typically a few milliseconds.
         *
         * <p>Logged, never consumed, and it must not be added to {@code leadS}: that constant is
         * the IMU-versus-wheels lag as fitted from a bag, which already contains this. It is here
         * so the next bag can say whether the fitted 20 ms is still the right number.
         */
        public double accelLatencyS = 0.0;

        public double[] odometryYawTimestamps = new double[0];
        public Rotation2d[] odometryYawPositions = new Rotation2d[0];
        public Rotation3d[] odometryRotations = new Rotation3d[0];
    }
}
