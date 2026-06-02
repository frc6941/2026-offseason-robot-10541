package lib.ironpulse.swerve.commands;

/** Stub — NT-tunable params for SwerveDriveToAllign command. */
public final class SwerveDriveToAllignParamsNT {
    public static final NTParam translationKp = new NTParam(4.0);
    public static final NTParam translationKi = new NTParam(0.0);
    public static final NTParam translationKiZone = new NTParam(0.0);
    public static final NTParam translationKd = new NTParam(0.0);
    public static final NTParam rotationKp = new NTParam(2.0);
    public static final NTParam rotationKi = new NTParam(0.0);
    public static final NTParam rotationKiZone = new NTParam(0.0);
    public static final NTParam rotationKd = new NTParam(0.0);
    public static final NTParam shiftingTerminate = new NTParam(0.3);
    public static final NTParam translationToleranceM = new NTParam(0.05);
    public static final NTParam rotationToleranceDeg = new NTParam(2.0);
    public static final NTParam translationStationaryMps = new NTParam(0.05);
    public static final NTParam rotationStationaryDegps = new NTParam(2.0);

    public static class NTParam {
        private final double value;
        public NTParam(double v) { this.value = v; }
        public double getValue() { return value; }
    }

    private SwerveDriveToAllignParamsNT() {}
}
