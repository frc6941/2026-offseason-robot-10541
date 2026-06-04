package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class FieldConstants {
    public static final double fieldLength = 17.548;
    public static final double fieldWidth = 8.052;

    public static class Hub {
        public static final double width = Units.inchesToMeters(47.0);
        public static final double innerHeight = Units.inchesToMeters(56.5);

        // Hub center on the blue alliance side (derived from AprilTag 26 position)
        public static final Translation3d TARGET =
                new Translation3d(4.022 + width / 2.0, fieldWidth / 2.0, innerHeight);

        // Opposite alliance hub target (derived from AprilTag 4 position)
        public static final Translation3d OPP_TARGET =
                new Translation3d(11.312 + width / 2.0, fieldWidth / 2.0, innerHeight);

        public static Translation2d getTarget2d() {
            return TARGET.toTranslation2d();
        }
    }

    private FieldConstants() {}
}
