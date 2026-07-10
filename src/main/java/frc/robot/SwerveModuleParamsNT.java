package frc.robot;

/** Read-only compatibility facade required by the vendored swerve module. */
public final class SwerveModuleParamsNT {
    // SwerveModule checks each module independently; allow every controller one startup update.
    private static final int MODULE_COUNT = 4;

    public static final class FixedDouble {
        private final double value;

        private FixedDouble(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
    }

    public static final class Drive {
        private static int startupUpdatesRemaining = MODULE_COUNT;

        public static final FixedDouble kP = new FixedDouble(SwerveModuleParams.Drive.kP);
        public static final FixedDouble kI = new FixedDouble(SwerveModuleParams.Drive.kI);
        public static final FixedDouble kD = new FixedDouble(SwerveModuleParams.Drive.kD);
        public static final FixedDouble kS = new FixedDouble(SwerveModuleParams.Drive.kS);
        public static final FixedDouble kV = new FixedDouble(SwerveModuleParams.Drive.kV);
        public static final FixedDouble kA = new FixedDouble(SwerveModuleParams.Drive.kA);

        public static boolean isAnyChanged() {
            if (startupUpdatesRemaining <= 0) {
                return false;
            }
            startupUpdatesRemaining--;
            return true;
        }

        private Drive() {}
    }

    public static final class Steer {
        private static int startupUpdatesRemaining = MODULE_COUNT;

        public static final FixedDouble kP = new FixedDouble(SwerveModuleParams.Steer.kP);
        public static final FixedDouble kI = new FixedDouble(SwerveModuleParams.Steer.kI);
        public static final FixedDouble kD = new FixedDouble(SwerveModuleParams.Steer.kD);
        public static final FixedDouble kS = new FixedDouble(SwerveModuleParams.Steer.kS);

        public static boolean isAnyChanged() {
            if (startupUpdatesRemaining <= 0) {
                return false;
            }
            startupUpdatesRemaining--;
            return true;
        }

        private Steer() {}
    }

    private SwerveModuleParamsNT() {}
}
