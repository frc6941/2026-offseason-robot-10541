package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Field-constant waypoints and dimensions for the 2026 FRC game (Reefscape).
 *
 * <p>All positions are defined for the <b>blue alliance</b> (left side in the field diagram).
 * Red-alliance equivalents are computed at runtime via {@link
 * lib.ironpulse.utils.AllianceFlipUtil#apply(Pose2d)}.
 *
 * <p>Pattern adapted from Team 6328's {@code AutoFieldConstants.java}.
 */
public final class FieldConstants {
    public static final double fieldLength = 17.548;
    public static final double fieldWidth = 8.052;

    // ========================================================================
    // Hub (target for AutoAimCommand and ShootingSuperstructure)
    // ========================================================================
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

    // ========================================================================
    // Robot start positions (blue alliance — ~0.76m from alliance wall)
    // Rotation2d() = facing toward the field (away from driver station)
    // ========================================================================
    public static class StartPositions {
        public static final Pose2d BLUE_LEFT =
                new Pose2d(0.76, 6.75, new Rotation2d());
        public static final Pose2d BLUE_CENTER =
                new Pose2d(0.76, fieldWidth / 2.0, new Rotation2d());
        public static final Pose2d BLUE_RIGHT =
                new Pose2d(0.76, 1.30, new Rotation2d());
    }

    // ========================================================================
    // Neutral zone — the open center region both alliances share
    // ========================================================================
    public static class NeutralZone {
        /** X coordinate where the blue-side robot enters the neutral zone. */
        public static final double BLUE_X_ENTRY = 4.50;
        /** X coordinate where the blue-side robot exits toward the red side. */
        public static final double BLUE_X_EXIT = 5.80;
        public static final double CENTER_Y = fieldWidth / 2.0;
    }

    // ========================================================================
    // Launch positions — poses from which the robot can shoot at the hub
    // Rotation2d.k180deg = facing the red side (toward the hub from blue)
    // ========================================================================
    public static class LaunchPositions {
        /** Shallow launch, close to the starting line. */
        public static final Pose2d BLUE_NEAR =
                new Pose2d(2.50, fieldWidth / 2.0, Rotation2d.k180deg);
        /** Deeper launch, further into the field. */
        public static final Pose2d BLUE_FAR =
                new Pose2d(4.00, fieldWidth / 2.0, Rotation2d.k180deg);
    }

    // ========================================================================
    // Reef approach poses (2026 Reefscape)
    // Distances and rotations are approximate — tune against field CAD.
    // ========================================================================
    public static class Reef {
        public static final Pose2d BLUE_AB =
                new Pose2d(3.20, 4.20, Rotation2d.k180deg);
        public static final Pose2d BLUE_CD =
                new Pose2d(3.20, 3.85, Rotation2d.k180deg);
        public static final Pose2d BLUE_EF =
                new Pose2d(3.65, 3.20, Rotation2d.kCCW_Pi_2);
        public static final Pose2d BLUE_GH =
                new Pose2d(4.35, 3.20, Rotation2d.kCCW_Pi_2);
        public static final Pose2d BLUE_IJ =
                new Pose2d(4.80, 3.85, new Rotation2d());
        public static final Pose2d BLUE_KL =
                new Pose2d(4.80, 4.20, new Rotation2d());
    }

    // ========================================================================
    // Coral station approach poses
    // ========================================================================
    public static class CoralStation {
        public static final Pose2d BLUE_LEFT =
                new Pose2d(0.76, 7.30, Rotation2d.fromDegrees(54.0));
        public static final Pose2d BLUE_RIGHT =
                new Pose2d(0.76, 0.75, Rotation2d.fromDegrees(-54.0));
    }

    private FieldConstants() {}
}
