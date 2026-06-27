// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package lib.ironpulse.utils;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import lib.ironpulse.math.obstacle.PolygonObstacle2d;

public class AllianceFlipUtil {
    private static final double FIELD_LENGTH_METERS = Units.inchesToMeters(690.876);
    private static final double FIELD_WIDTH_METERS = Units.inchesToMeters(317.0);

    public static double applyX(double x) {
        return shouldFlip() ? FIELD_LENGTH_METERS - x : x;
    }

    public static double applyY(double y) {
        return shouldFlip() ? FIELD_WIDTH_METERS - y : y;
    }

    public static Translation2d apply(Translation2d translation) {
        return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
    }

    public static Rotation2d apply(Rotation2d rotation) {
        return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
    }

    public static Rotation3d apply(Rotation3d rotation) {
        return shouldFlip() ? rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI)) : rotation;
    }

    public static Pose2d apply(Pose2d p) {
        if (shouldFlip()) {
            Translation2d t = apply(p.getTranslation());
            Rotation2d r = apply(p.getRotation());
            return new Pose2d(t, r);
        }
        return p;
    }

    public static Translation3d apply(Translation3d t3) {
        if (shouldFlip()) {
            double x = FIELD_LENGTH_METERS - t3.getX();
            double y = FIELD_WIDTH_METERS - t3.getY();
            return new Translation3d(x, y, t3.getZ());
        }
        return t3;
    }

    public static PolygonObstacle2d apply(PolygonObstacle2d pN) {

        if (shouldFlip()) {
            Translation2d[] cPt = new Translation2d[pN.cornerPoints.length];
            for (int i = 0; i < pN.cornerPoints.length; i++) {
                cPt[i] = apply(pN.cornerPoints[i]);
            }
            return new PolygonObstacle2d(cPt);
        } else return pN;
    }

    public static Pose3d apply(Pose3d pose) {
        return new Pose3d(apply(pose.getTranslation()), apply(pose.getRotation()));
    }

    public static boolean shouldFlip() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }
}
