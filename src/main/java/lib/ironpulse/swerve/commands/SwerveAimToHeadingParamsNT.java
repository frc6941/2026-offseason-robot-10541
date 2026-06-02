package lib.ironpulse.swerve.commands;

/** Stub — NT-tunable params for SwerveAimToHeading command. */
public final class SwerveAimToHeadingParamsNT {
    public static final NTParam rotationKp = new NTParam(2.0);
    public static final NTParam rotationKi = new NTParam(0.0);
    public static final NTParam rotationKd = new NTParam(0.0);

    public static class NTParam {
        private final double value;
        public NTParam(double v) { this.value = v; }
        public double getValue() { return value; }
    }

    private SwerveAimToHeadingParamsNT() {}
}
