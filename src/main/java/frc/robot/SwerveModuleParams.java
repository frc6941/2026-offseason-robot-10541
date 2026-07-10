package frc.robot;

/**
 * Fixed swerve module PID/FF gains.
 *
 * <p>{@link SwerveModuleParamsNT} is a read-only compatibility facade for the vendored swerve code.
 *
 * <p>TODO: tune drive + steer gains on the real robot.
 */
public final class SwerveModuleParams {
    public static final class Drive {
        public static final double kP = 6.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.65;
        public static final double kA = 0.05;
    }

    public static final class Steer {
        public static final double kP = 6.0;
        public static final double kI = 0.0;
        public static final double kD = 0.1;
        public static final double kS = 0.0;
    }
}
