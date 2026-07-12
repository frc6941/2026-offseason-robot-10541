package frc.robot;

import lib.ntext.NTParameter;

/**
 * NT-tunable swerve module PID/FF gains. The {@code @NTParameter} processor generates {@code
 * SwerveModuleParamsNT} (wrapper fields + {@code isAnyChanged()} per group) from this class; the
 * vendored swerve code reads that generated facade. Live updates are gated by {@link
 * RobotConstants#ENABLE_NT_PARAMS}.
 *
 * <p>TODO: tune drive + steer gains on the real robot.
 */
@NTParameter(tableName = "Params/SwerveModule")
public final class SwerveModuleParams {
        public static final class Drive {
            static final double kP = 6;
            static final double kI = 0;
            static final double kD = 0;
            static final double kS = 0;
            // CTRE Slot0 kV for VelocityTorqueCurrentFOC with motor velocity units (rotor rps):
            // kV ~= 12V / (5800rpm / 60) = 0.124
            static final double kV = 0.136;
            static final double kA = 0.05;
            static final boolean isBrake = true;
        }

        public static final class Steer {
            static final double kP = 60;
            static final double kI = 0;
            static final double kD = 0.1;
            static final double kS = 0;
            static final boolean isBrake = true;
        }
    
}
