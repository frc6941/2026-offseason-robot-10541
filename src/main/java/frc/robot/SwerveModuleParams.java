package frc.robot;

import lib.ntext.NTParameter;

/**
 * NT-tunable swerve module PID/FF gains. The {@code @NTParameter} processor generates {@code
 * frc.robot.SwerveModuleParamsNT} (nested {@code Drive}/{@code Steer} with per-group {@code
 * isAnyChanged()}), which {@code lib.ironpulse.swerve.SwerveModule} consumes.
 *
 * <p>This MUST live in package {@code frc.robot} because the vendored lib imports {@code
 * frc.robot.SwerveModuleParamsNT} directly. Keep it here (not in a sub-package) so future lib
 * copies stay drop-in.
 *
 * <p>TODO: tune drive + steer gains on the real robot.
 */
@NTParameter(tableName = "Params/SwerveModule")
public final class SwerveModuleParams {
    public static final class Drive {
        public static final double kP = 10.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.136;
        public static final double kA = 0.05;
    }

    public static final class Steer {
        public static final double kP = 10.0;
        public static final double kI = 0.0;
        public static final double kD = 0.1;
        public static final double kS = 0.0;
    }
}
