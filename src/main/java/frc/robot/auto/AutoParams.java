package frc.robot.auto;

/**
 * Static tuning constants for autonomous. Mirrors the competition robot's {@code AutoParamsNT} but
 * as plain {@code static final} constants (this robot does not use NetworkTables params). Edit the
 * numbers here and redeploy.
 */
public final class AutoParams {
    private AutoParams() {}

    /**
     * Gains for PathPlanner path following ({@link
     * com.pathplanner.lib.commands.FollowPathCommand}).
     */
    public static final class AutoPathParams {
        public static final double kpStrave = 5.0;
        public static final double kiStrave = 0.0;
        public static final double kdStrave = 0.0;
        public static final double kpSpin = 5.0;
        public static final double kiSpin = 0.0;
        public static final double kdSpin = 0.0;

        private AutoPathParams() {}
    }

    /**
     * Gains + tolerances for point moves ({@code SwerveDriveToPose} / {@code SwerveAimToHeading}).
     */
    public static final class AutoPoseParams {
        public static final double kpStrave = 4.0;
        public static final double kiStrave = 0.0;
        public static final double kdStrave = 0.0;
        public static final double kpSpin = 5.0;
        public static final double kiSpin = 0.0;
        public static final double kdSpin = 0.0;
        public static final double tolerancePositionM = 0.05;
        public static final double toleranceHeadingDeg = 2.0;

        private AutoPoseParams() {}
    }

    /** Shooting behaviour for the non-turret "drive to a pose and empty the hopper" flow. */
    public static final class AutoShootParams {
        // How long to run the feed once at the shoot pose. Size this to empty a full hopper.
        public static final double feedSeconds = 3.0;
        // Spin-up settling budget before the feed opens.
        public static final double readyTimeoutSeconds = 1.5;

        private AutoShootParams() {}
    }
}
