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
    public static final double fieldLength = 16.541;
    public static final double fieldWidth = 8.069;

    public static final Translation2d fieldCenter =
            new Translation2d(fieldLength / 2.0, fieldWidth / 2.0);
    private static final double HUB_WIDTH = Units.inchesToMeters(47.0);

    public static class RobotFootprint {
        public static final double fullApothemX = 0.483253;
        public static final double fullApothemY = 0.483253;

        private RobotFootprint() {}
    }

    public static class AprilTags {
        public static final double HUB_NEAR_FACE_X = 4.0218614;
        public static final double HUB_FAR_FACE_X = 5.2291742;
        public static final double OPP_HUB_NEAR_FACE_X = 11.3118646;
        public static final double OPP_ALLIANCE_ZONE_X = 12.5191774;
        public static final double BLUE_OUTPOST_X = 0.007747;
        public static final double BLUE_OUTPOST_CENTER_Y = 0.6659626;
        public static final double BLUE_TOWER_CENTER_Y = 3.7457126;

        private AprilTags() {}
    }

    public static class LinesVertical {
        public static final double center = fieldLength / 2.0;
        public static final double starting = AprilTags.HUB_NEAR_FACE_X;
        public static final double allianceZone = starting;
        public static final double hubCenter = AprilTags.HUB_NEAR_FACE_X + HUB_WIDTH / 2.0;
        public static final double neutralZoneNear = center - Units.inchesToMeters(120.0);
        public static final double neutralZoneFar = center + Units.inchesToMeters(120.0);
        public static final double oppHubCenter = AprilTags.OPP_HUB_NEAR_FACE_X + HUB_WIDTH / 2.0;
        public static final double oppAllianceZone = AprilTags.OPP_ALLIANCE_ZONE_X;

        private LinesVertical() {}
    }

    public static class LinesHorizontal {
        public static final double center = fieldWidth / 2.0;

        public static final double rightBumpStart = Hub.nearRightCorner.getY();
        public static final double rightBumpEnd = rightBumpStart - RightBump.width;
        public static final double rightBumpMiddle = (rightBumpStart + rightBumpEnd) / 2.0;
        public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);
        public static final double rightTrenchOpenEnd = 0.0;
        public static final double rightTrenchMiddle =
                (rightTrenchOpenStart + rightTrenchOpenEnd) / 2.0;

        public static final double leftBumpEnd = Hub.nearLeftCorner.getY();
        public static final double leftBumpStart = leftBumpEnd + LeftBump.width;
        public static final double leftBumpMiddle = (leftBumpStart + leftBumpEnd) / 2.0;
        public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);
        public static final double leftTrenchOpenStart = fieldWidth;
        public static final double leftTrenchMiddle =
                (leftTrenchOpenEnd + leftTrenchOpenStart) / 2.0;

        private LinesHorizontal() {}
    }

    // ========================================================================
    // Hub (target for AutoAimCommand and ShootingSuperstructure)
    // ========================================================================
    public static class Hub {
        public static final double width = HUB_WIDTH;
        public static final double height = Units.inchesToMeters(72.0);
        public static final double innerHeight = Units.inchesToMeters(56.5);

        public static final Translation3d TARGET =
                new Translation3d(LinesVertical.hubCenter, fieldWidth / 2.0, innerHeight);

        public static final Translation3d OPP_TARGET =
                new Translation3d(LinesVertical.oppHubCenter, fieldWidth / 2.0, innerHeight);

        public static final Translation2d nearLeftCorner =
                new Translation2d(AprilTags.HUB_NEAR_FACE_X, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d nearRightCorner =
                new Translation2d(AprilTags.HUB_NEAR_FACE_X, fieldWidth / 2.0 - width / 2.0);
        public static final Translation2d farLeftCorner =
                new Translation2d(AprilTags.HUB_FAR_FACE_X, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d farRightCorner =
                new Translation2d(AprilTags.HUB_FAR_FACE_X, fieldWidth / 2.0 - width / 2.0);

        public static Translation2d getTarget2d() {
            return TARGET.toTranslation2d();
        }

        private Hub() {}
    }

    public static class LeftBump {
        public static final double width = Units.inchesToMeters(73.0);
        public static final double height = Units.inchesToMeters(6.513);
        public static final double depth = Units.inchesToMeters(44.4);

        public static final Translation2d nearLeftCorner =
                Hub.nearLeftCorner.plus(new Translation2d(0.0, width));
        public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
        public static final Translation2d farLeftCorner =
                Hub.farLeftCorner.plus(new Translation2d(0.0, width));
        public static final Translation2d farRightCorner = Hub.farLeftCorner;

        private LeftBump() {}
    }

    public static class RightBump {
        public static final double width = Units.inchesToMeters(73.0);
        public static final double height = Units.inchesToMeters(6.513);
        public static final double depth = Units.inchesToMeters(44.4);

        public static final Translation2d nearLeftCorner = Hub.nearRightCorner;
        public static final Translation2d nearRightCorner =
                Hub.nearRightCorner.minus(new Translation2d(0.0, width));
        public static final Translation2d farLeftCorner = Hub.farRightCorner;
        public static final Translation2d farRightCorner =
                Hub.farRightCorner.minus(new Translation2d(0.0, width));

        private RightBump() {}
    }

    public static class LeftTrench {
        public static final double width = Units.inchesToMeters(65.65);
        public static final double depth = Units.inchesToMeters(47.0);
        public static final double height = Units.inchesToMeters(40.25);
        public static final double openingWidth = Units.inchesToMeters(50.34);
        public static final double openingHeight = Units.inchesToMeters(22.25);

        public static final Translation3d openingTopLeft =
                new Translation3d(LinesVertical.hubCenter, fieldWidth, openingHeight);
        public static final Translation3d openingTopRight =
                new Translation3d(
                        LinesVertical.hubCenter, fieldWidth - openingWidth, openingHeight);
        public static final Translation2d center =
                openingTopLeft
                        .toTranslation2d()
                        .interpolate(openingTopRight.toTranslation2d(), 0.5);

        private LeftTrench() {}
    }

    public static class RightTrench {
        public static final double width = Units.inchesToMeters(65.65);
        public static final double depth = Units.inchesToMeters(47.0);
        public static final double height = Units.inchesToMeters(40.25);
        public static final double openingWidth = Units.inchesToMeters(50.34);
        public static final double openingHeight = Units.inchesToMeters(22.25);

        public static final Translation3d openingTopLeft =
                new Translation3d(LinesVertical.hubCenter, openingWidth, openingHeight);
        public static final Translation3d openingTopRight =
                new Translation3d(LinesVertical.hubCenter, 0.0, openingHeight);
        public static final Translation2d center =
                openingTopLeft
                        .toTranslation2d()
                        .interpolate(openingTopRight.toTranslation2d(), 0.5);

        private RightTrench() {}
    }

    public static class Depot {
        public static final double width = Units.inchesToMeters(42.0);
        public static final double depth = Units.inchesToMeters(27.0);
        public static final double height = Units.inchesToMeters(1.125);
        public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

        public static final Translation3d depotCenter =
                new Translation3d(depth, (fieldWidth / 2.0) + distanceFromCenterY, height);
        public static final Translation3d leftCorner =
                new Translation3d(depth, depotCenter.getY() + width / 2.0, height);
        public static final Translation3d rightCorner =
                new Translation3d(depth, depotCenter.getY() - width / 2.0, height);

        private Depot() {}
    }

    public static class Tower {
        public static final double width = Units.inchesToMeters(49.25);
        public static final double depth = Units.inchesToMeters(45.0);
        public static final double height = Units.inchesToMeters(78.25);
        public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
        public static final double frontFaceX = Units.inchesToMeters(43.51);
        public static final double uprightHeight = Units.inchesToMeters(72.1);

        public static final Translation2d centerPoint =
                new Translation2d(frontFaceX, AprilTags.BLUE_TOWER_CENTER_Y);
        public static final Translation2d leftUpright =
                new Translation2d(
                        frontFaceX,
                        AprilTags.BLUE_TOWER_CENTER_Y
                                + innerOpeningWidth / 2.0
                                + Units.inchesToMeters(0.75));
        public static final Translation2d rightUpright =
                new Translation2d(
                        frontFaceX,
                        AprilTags.BLUE_TOWER_CENTER_Y
                                - innerOpeningWidth / 2.0
                                - Units.inchesToMeters(0.75));

        private Tower() {}
    }

    public static class Outpost {
        public static final double width = Units.inchesToMeters(31.8);
        public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
        public static final double height = Units.inchesToMeters(7.0);

        public static final Translation2d centerPoint =
                new Translation2d(AprilTags.BLUE_OUTPOST_X, AprilTags.BLUE_OUTPOST_CENTER_Y);

        private Outpost() {}
    }

    public static class FuelPool {
        public static final double width = Units.inchesToMeters(181.9);
        public static final double depth = Units.inchesToMeters(71.9);

        public static final Translation2d nearLeftCorner =
                new Translation2d(fieldLength / 2.0 - depth / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d nearRightCorner =
                new Translation2d(fieldLength / 2.0 - depth / 2.0, fieldWidth / 2.0 - width / 2.0);
        public static final Translation2d leftCenter =
                new Translation2d(fieldLength / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d rightCenter =
                new Translation2d(fieldLength / 2.0, fieldWidth / 2.0 - width / 2.0);

        private FuelPool() {}
    }

    public static Pose2d aimedAtBlueHub(Translation2d robotTranslation) {
        return new Pose2d(robotTranslation, Hub.getTarget2d().minus(robotTranslation).getAngle());
    }

    // ========================================================================
    // Robot start positions (blue alliance — ~0.76m from alliance wall)
    // Rotation2d() = facing toward the field (away from driver station)
    // ========================================================================
    public static class StartPositions {
        public static final Pose2d BLUE_LEFT = new Pose2d(0.76, 6.75, new Rotation2d());
        public static final Pose2d BLUE_CENTER =
                new Pose2d(0.76, fieldWidth / 2.0, new Rotation2d());
        public static final Pose2d BLUE_RIGHT = new Pose2d(0.76, 1.30, new Rotation2d());
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
    // Pass targets — when the robot is in the neutral zone it lobs the ball back
    // toward its own alliance zone instead of shooting the hub. Two points mirrored
    // across the field's horizontal centerline sit just off the driver-station wall;
    // AutoAimCommand aims the shooter at whichever is on the robot's own Y-half, so
    // the pass stays down its sideline and never crosses through the hub.
    // Blue-alliance frame — red equivalents come from AllianceFlipUtil at runtime.
    // ========================================================================
    public static class PassTargets {
        /** Distance from the blue wall — near the driver station (≈ LaunchPositions.BLUE_NEAR). */
        public static final double X = 2.50;

        /** Off-center lateral offset; matches the alliance-zone corners, well clear of the hub. */
        public static final Translation2d BLUE_LEFT = new Translation2d(X, 6.75);

        public static final Translation2d BLUE_RIGHT = new Translation2d(X, 1.30);

        private PassTargets() {}
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
        public static final Pose2d BLUE_AB = new Pose2d(3.20, 4.20, Rotation2d.k180deg);
        public static final Pose2d BLUE_CD = new Pose2d(3.20, 3.85, Rotation2d.k180deg);
        public static final Pose2d BLUE_EF = new Pose2d(3.65, 3.20, Rotation2d.kCCW_Pi_2);
        public static final Pose2d BLUE_GH = new Pose2d(4.35, 3.20, Rotation2d.kCCW_Pi_2);
        public static final Pose2d BLUE_IJ = new Pose2d(4.80, 3.85, new Rotation2d());
        public static final Pose2d BLUE_KL = new Pose2d(4.80, 4.20, new Rotation2d());
    }

    // ========================================================================
    // Coral station approach poses
    // ========================================================================
    public static class CoralStation {
        public static final Pose2d BLUE_LEFT = new Pose2d(0.76, 7.30, Rotation2d.fromDegrees(54.0));
        public static final Pose2d BLUE_RIGHT =
                new Pose2d(0.76, 0.75, Rotation2d.fromDegrees(-54.0));
    }

    private FieldConstants() {}
}
