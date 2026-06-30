package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;

/**
 * Blue-alliance field reference points for pathfinding autos.
 *
 * <p>These poses are written in the global blue-origin field frame. Commands should use the flipped
 * PathPlanner pathfind APIs so the same definitions work on both alliances.
 */
public final class AutoPoints {
    private static final double WALL_TO_ROBOT_CENTER_X =
            FieldConstants.Outpost.centerPoint.getX() + FieldConstants.RobotFootprint.fullApothemX;
    private static final double MID_EDGE_CLEARANCE_METERS = 0.35;

    private static Translation2d verticalFlip(Translation2d blueTranslation) {
        return new Translation2d(
                blueTranslation.getX(),
                FieldConstants.fieldWidth - blueTranslation.getY());
    }

    private static Pose2d verticalFlip(Pose2d bluePose) {
        return new Pose2d(
                verticalFlip(bluePose.getTranslation()),
                bluePose.getRotation().unaryMinus());
    }

    public static class Trench {
        public static final Translation2d LEFT_START =
                new Translation2d(
                        FieldConstants.LinesVertical.starting + FieldConstants.RobotFootprint.fullApothemX,
                        FieldConstants.LinesHorizontal.leftTrenchMiddle);
        public static final Translation2d RIGHT_START = verticalFlip(LEFT_START);

        public static final Translation2d LEFT_START_OFFSET =
                new Translation2d(
                        FieldConstants.LinesVertical.starting - FieldConstants.RobotFootprint.fullApothemX,
                        FieldConstants.LinesHorizontal.leftTrenchMiddle);
        public static final Translation2d RIGHT_START_OFFSET = verticalFlip(LEFT_START_OFFSET);

        public static final Translation2d LEFT_ENTRY = LEFT_START_OFFSET;
        public static final Translation2d RIGHT_ENTRY = verticalFlip(LEFT_ENTRY);

        public static final Translation2d LEFT_BEFORE_BAR =
                new Translation2d(
                        (FieldConstants.LinesVertical.allianceZone + FieldConstants.LinesVertical.neutralZoneNear)
                                        / 2.0
                                - Units.inchesToMeters(2.95) / 2.0
                                - FieldConstants.RobotFootprint.fullApothemX,
                        FieldConstants.LinesHorizontal.leftTrenchMiddle);
        public static final Translation2d RIGHT_BEFORE_BAR = verticalFlip(LEFT_BEFORE_BAR);

        public static final Translation2d LEFT_CLEAR =
                new Translation2d(
                        FieldConstants.LinesVertical.starting
                                + FieldConstants.LeftTrench.depth
                                + FieldConstants.RobotFootprint.fullApothemX
                                + 0.2,
                        FieldConstants.LinesHorizontal.leftTrenchMiddle);
        public static final Translation2d RIGHT_CLEAR = verticalFlip(LEFT_CLEAR);

        private Trench() {}
    }

    public static class Bump {
        public static final Translation2d LEFT_INNER =
                new Translation2d(
                        FieldConstants.LinesVertical.starting - FieldConstants.RobotFootprint.fullApothemX,
                        FieldConstants.LinesHorizontal.leftBumpMiddle);
        public static final Translation2d RIGHT_INNER = verticalFlip(LEFT_INNER);

        public static final Translation2d LEFT_OUTER =
                new Translation2d(
                        FieldConstants.LinesVertical.neutralZoneNear + FieldConstants.RobotFootprint.fullApothemX,
                        FieldConstants.LinesHorizontal.leftBumpMiddle);
        public static final Translation2d RIGHT_OUTER = verticalFlip(LEFT_OUTER);

        private Bump() {}
    }

    public static class Hub {
        public static final Translation2d CENTER_START =
                new Translation2d(
                        FieldConstants.LinesVertical.starting - FieldConstants.RobotFootprint.fullApothemX,
                        FieldConstants.LinesHorizontal.center);

        private Hub() {}
    }

    public static class NeutralZone {
        public static final Translation2d LEFT_EDGE =
                new Translation2d(
                        FieldConstants.FuelPool.leftCenter.getX(),
                        FieldConstants.fieldWidth
                                - FieldConstants.RobotFootprint.fullApothemY
                                - MID_EDGE_CLEARANCE_METERS);
        public static final Translation2d RIGHT_EDGE =
                new Translation2d(
                        FieldConstants.FuelPool.rightCenter.getX(),
                        FieldConstants.RobotFootprint.fullApothemY + MID_EDGE_CLEARANCE_METERS);

        public static final Translation2d LEFT_CENTER = FieldConstants.FuelPool.leftCenter;
        public static final Translation2d RIGHT_CENTER = FieldConstants.FuelPool.rightCenter;

        public static final Translation2d LEFT_CONSERVATIVE =
                new Translation2d(
                        FieldConstants.FuelPool.nearLeftCorner.getX(),
                        FieldConstants.FuelPool.leftCenter.getY() - 0.35);
        public static final Translation2d RIGHT_CONSERVATIVE =
                new Translation2d(
                        FieldConstants.FuelPool.nearRightCorner.getX(),
                        FieldConstants.FuelPool.rightCenter.getY() + 0.35);

        public static final Translation2d LEFT_FLIGHTLESS =
                new Translation2d(
                        FieldConstants.LinesVertical.neutralZoneNear
                                + FieldConstants.RobotFootprint.fullApothemX
                                + 0.3,
                        FieldConstants.LinesHorizontal.leftBumpMiddle);
        public static final Translation2d RIGHT_FLIGHTLESS = verticalFlip(LEFT_FLIGHTLESS);
        public static final Translation2d LEFT_FLIGHTLESS_WIDE =
                new Translation2d(LEFT_FLIGHTLESS.getX(), LEFT_EDGE.getY());
        public static final Translation2d RIGHT_FLIGHTLESS_WIDE = verticalFlip(LEFT_FLIGHTLESS_WIDE);

        public static final Translation2d LEFT_DAVIS =
                new Translation2d(FieldConstants.fieldCenter.getX() + 0.75, LEFT_EDGE.getY());
        public static final Translation2d RIGHT_DAVIS =
                new Translation2d(FieldConstants.fieldCenter.getX() + 0.75, RIGHT_EDGE.getY());

        public static final Translation2d LEFT_CORIOLIS =
                new Translation2d(FieldConstants.fieldCenter.getX() - 0.75, LEFT_CENTER.getY());
        public static final Translation2d RIGHT_CORIOLIS =
                new Translation2d(FieldConstants.fieldCenter.getX() - 0.75, RIGHT_CENTER.getY());

        public static final Translation2d LEFT_CENTER_FORWARD =
                new Translation2d(FieldConstants.fieldCenter.getX() - 0.35, LEFT_CENTER.getY());
        public static final Translation2d RIGHT_CENTER_FORWARD =
                new Translation2d(FieldConstants.fieldCenter.getX() - 0.35, RIGHT_CENTER.getY());

        public static final Translation2d LEFT_SALESMAN_TURN =
                new Translation2d(
                        FieldConstants.FuelPool.leftCenter.getX() - 0.8,
                        FieldConstants.FuelPool.leftCenter.getY() - 1.0);
        public static final Translation2d RIGHT_SALESMAN_TURN = verticalFlip(LEFT_SALESMAN_TURN);

        public static final double WAVE_AMPLITUDE_METERS = 0.65;
        public static final double FLIGHTLESS_WAVE_AMPLITUDE_METERS = 0.35;

        public static double waveOffset(double progress, double amplitudeMeters) {
            return amplitudeMeters * Math.sin(progress * 2.0 * Math.PI);
        }

        private NeutralZone() {}
    }

    public static class Depot {
        public static final Translation2d LEFT_THROUGH =
                new Translation2d(
                        FieldConstants.Depot.depth / 2.0 + 0.2,
                        FieldConstants.Depot.leftCorner.getY()
                                + FieldConstants.RobotFootprint.fullApothemX
                                + 0.1);
        public static final Translation2d RIGHT_THROUGH =
                new Translation2d(
                        FieldConstants.Depot.depth / 2.0 + 0.2,
                        FieldConstants.Depot.rightCorner.getY()
                                - FieldConstants.RobotFootprint.fullApothemX
                                - 0.1);

        private Depot() {}
    }

    public static class Tower {
        public static final Translation2d LEFT_THROUGH =
                new Translation2d(
                        FieldConstants.Tower.frontFaceX / 2.0,
                        FieldConstants.Tower.leftUpright.getY()
                                + FieldConstants.RobotFootprint.fullApothemX
                                + 0.25);
        public static final Translation2d RIGHT_THROUGH =
                new Translation2d(
                        FieldConstants.Tower.frontFaceX / 2.0,
                        FieldConstants.Tower.rightUpright.getY()
                                - FieldConstants.RobotFootprint.fullApothemX
                                - 0.25);

        public static final Translation2d LEFT_OUTSIDE =
                FieldConstants.Tower.leftUpright.plus(
                        new Translation2d(
                                FieldConstants.RobotFootprint.fullApothemX + 0.3,
                                FieldConstants.RobotFootprint.fullApothemX + 0.3));
        public static final Translation2d RIGHT_OUTSIDE =
                FieldConstants.Tower.rightUpright.plus(
                        new Translation2d(
                                FieldConstants.RobotFootprint.fullApothemX + 0.3,
                                -FieldConstants.RobotFootprint.fullApothemX - 0.3));

        private Tower() {}
    }

    public static class Climb {
        public static final Pose2d LEFT =
                new Pose2d(
                        FieldConstants.Tower.leftUpright.plus(
                                new Translation2d(0.0, FieldConstants.RobotFootprint.fullApothemX)),
                        Rotation2d.kZero);
        public static final Pose2d RIGHT =
                new Pose2d(
                        FieldConstants.Tower.rightUpright.plus(
                                new Translation2d(0.0, -FieldConstants.RobotFootprint.fullApothemX)),
                        Rotation2d.kPi);

        public static final Pose2d LEFT_OFFSET =
                new Pose2d(
                        FieldConstants.Tower.leftUpright.plus(
                                new Translation2d(0.0, FieldConstants.RobotFootprint.fullApothemX + 0.6)),
                        Rotation2d.kZero);
        public static final Pose2d RIGHT_OFFSET =
                new Pose2d(
                        FieldConstants.Tower.rightUpright.plus(
                                new Translation2d(0.0, -FieldConstants.RobotFootprint.fullApothemX - 0.6)),
                        Rotation2d.kPi);

        private Climb() {}
    }

    public static class Launch {
        public static final Pose2d LEFT_TOWER =
                FieldConstants.aimedAtBlueHub(Climb.LEFT.getTranslation().plus(new Translation2d(1.5, 0.5)));
        public static final Pose2d RIGHT_TOWER =
                FieldConstants.aimedAtBlueHub(Climb.RIGHT.getTranslation().plus(new Translation2d(1.5, -0.5)));

        public static final Pose2d LEFT_BUMP =
                FieldConstants.aimedAtBlueHub(
                        new Translation2d(
                                FieldConstants.LinesVertical.starting - 0.7,
                                FieldConstants.LinesHorizontal.leftBumpMiddle));
        public static final Pose2d RIGHT_BUMP = verticalFlip(LEFT_BUMP);

        public static final Pose2d LEFT_TRENCH =
                FieldConstants.aimedAtBlueHub(
                        new Translation2d(
                                        FieldConstants.LinesVertical.starting
                                                - Math.hypot(
                                                        FieldConstants.RobotFootprint.fullApothemX,
                                                        FieldConstants.RobotFootprint.fullApothemY / 2.0),
                                        FieldConstants.LinesHorizontal.leftTrenchMiddle)
                                .plus(new Translation2d(-0.5, -0.3)));
        public static final Pose2d RIGHT_TRENCH = verticalFlip(LEFT_TRENCH);

        private Launch() {}
    }

    public static final Pose2d DEPOT_X_START =
            new Pose2d(
                    WALL_TO_ROBOT_CENTER_X + 1.0,
                    FieldConstants.Depot.depotCenter.getY(),
                    Rotation2d.kPi);
    public static final Pose2d DEPOT_X_END =
            new Pose2d(
                    WALL_TO_ROBOT_CENTER_X,
                    FieldConstants.Depot.depotCenter.getY(),
                    Rotation2d.kPi);

    public static final Pose2d DEPOT_Y_START =
            new Pose2d(WALL_TO_ROBOT_CENTER_X, Depot.LEFT_THROUGH.getY(), Rotation2d.fromDegrees(-90.0));
    public static final Pose2d DEPOT_Y_END =
            new Pose2d(
                    WALL_TO_ROBOT_CENTER_X,
                    FieldConstants.Depot.depotCenter.getY(),
                    Rotation2d.fromDegrees(-90.0));

    public static final Pose2d START_LEFT = new Pose2d(Bump.LEFT_INNER, Rotation2d.kZero);
    public static final Pose2d START_RIGHT = new Pose2d(Bump.RIGHT_INNER, Rotation2d.kZero);

    public static final Translation2d MID_LEFT = NeutralZone.LEFT_EDGE;
    public static final Translation2d MID_RIGHT = NeutralZone.RIGHT_EDGE;

    public static final Pose2d OUTPOST =
            new Pose2d(WALL_TO_ROBOT_CENTER_X, FieldConstants.Outpost.centerPoint.getY(), Rotation2d.kZero);

    public static final Pose2d OUTPOST_APPROACH =
            new Pose2d(OUTPOST.getX() + 1.0, OUTPOST.getY(), Rotation2d.kZero);

    private AutoPoints() {}
}
