package frc.robot.auto;

import lib.ntext.NTParameter;

/**
 * NT-tunable autonomous tuning parameters. Each nested class is turned into a {@code …NT} wrapper
 * by the {@code @NTParameter} processor; consumers read {@code …NT.field.getValue()}. Live updates
 * are gated by {@link frc.robot.RobotConstants#ENABLE_NT_PARAMS}.
 */
public final class AutoParams {
    private AutoParams() {}

    /**
     * Gains for PathPlanner path following ({@link
     * com.pathplanner.lib.commands.FollowPathCommand}).
     */
    @NTParameter(tableName = "Params/AutoPath")
    public static final class AutoPathParams {
        public static final double kpStrave = 1.6;
        public static final double kiStrave = 0.0;
        public static final double kdStrave = 0.0;
        public static final double kpSpin = 0.2;
        public static final double kiSpin = 0.0;
        public static final double kdSpin = 0.0;

        private AutoPathParams() {}
    }

    /**
     * Gains + tolerances for point moves ({@code SwerveDriveToPose} / {@code SwerveAimToHeading}).
     * Field names/defaults mirror {@code SwerveDriveToPose.SwerveDriveToPoseParams} so the two stay
     * in lockstep at a glance.
     */
    @NTParameter(tableName = "Params/AutoPose")
    public static final class AutoPoseParams {
        public static final double translationKp = 2.7;
        public static final double translationKi = 0.0;
        public static final double translationKd = 0.0;
        public static final double rotationKp = 1.2;
        public static final double rotationKi = 0.0;
        public static final double rotationKd = 0.0;
        public static final double tolerancePositionM = 0.05;
        public static final double toleranceHeadingDeg = 2.0;

        private AutoPoseParams() {}
    }

    /** Shooting behaviour for the non-turret "drive to a pose and empty the hopper" flow. */
    @NTParameter(tableName = "Params/AutoShoot")
    public static final class AutoShootParams {
        // How long to run the feed once at the shoot pose. Size this to empty a full hopper.
        // Total auto shot ≈ FEED_DELAY_AFTER_UPPER_READY (0.2s) + feedSeconds, because the flywheel
        // is pre-spun during the drive in (see ShootingSuperstructure.spinUpForShot), so there's no
        // spin-up wait. 1.5 keeps the whole shot within ~2s.
        public static final double feedSeconds = 1.5;
        // Max time to wait for the flywheel to reach speed before giving up and NOT feeding. With
        // pre-spin the drum is at speed on arrival, so this is rarely used; it only caps the wait.
        public static final double readyTimeoutSeconds = 2.5;

        private AutoShootParams() {}
    }
}
