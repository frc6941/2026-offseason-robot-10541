package lib.ironpulse.swerve.commands;

/** Stub — NT-tunable params for SwerveDriveToPose command. */
public final class SwerveDriveToPoseParamsNT {
    public static final NTParam translationKp = new NTParam(4.0);
    public static final NTParam translationKi = new NTParam(0.0);
    public static final NTParam translationKd = new NTParam(0.0);
    public static final NTParam rotationKp = new NTParam(2.0);
    public static final NTParam rotationKi = new NTParam(0.0);
    public static final NTParam rotationKd = new NTParam(0.0);

    public static class NTParam {
        private final double value;
        public NTParam(double v) { this.value = v; }
        public double getValue() { return value; }
    }

    private SwerveDriveToPoseParamsNT() {}
}
