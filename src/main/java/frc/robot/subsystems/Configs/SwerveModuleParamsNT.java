package frc.robot.subsystems.Configs;

/**
 * Stub — NT-tunable swerve module PID parameters.
 * Replace with real NTParameter-generated version when tuning is needed.
 */
public final class SwerveModuleParamsNT {
    public static final PidGroup Drive = new PidGroup();
    public static final SteerPidGroup Steer = new SteerPidGroup();

    public static class PidGroup {
        public final NTParam kP = new NTParam(0.0);
        public final NTParam kI = new NTParam(0.0);
        public final NTParam kD = new NTParam(0.0);
        public final NTParam kS = new NTParam(0.0);
        public final NTParam kV = new NTParam(0.0);
        public final NTParam kA = new NTParam(0.0);
        public boolean isAnyChanged() { return false; }
    }

    public static class SteerPidGroup {
        public final NTParam kP = new NTParam(0.0);
        public final NTParam kI = new NTParam(0.0);
        public final NTParam kD = new NTParam(0.0);
        public final NTParam kS = new NTParam(0.0);
        public boolean isAnyChanged() { return false; }
    }

    public static class NTParam {
        private final double value;
        public NTParam(double v) { this.value = v; }
        public double getValue() { return value; }
    }

    private SwerveModuleParamsNT() {}
}
