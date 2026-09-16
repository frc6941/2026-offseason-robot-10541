package lib.ironpulse.swerve;

/**
 * A chassis acceleration reference, expressed in the ROBOT frame in SI units — the acceleration
 * counterpart of {@link edu.wpi.first.math.kinematics.ChassisSpeeds}.
 *
 * <p>This exists so a planner can state the acceleration it actually intends rather than leaving
 * the consumer to differentiate a velocity command. Differentiating is safe for a trajectory, whose
 * velocity is a planned function of time; it is not safe for the output of a position feedback
 * loop, where every pose correction becomes a step and its derivative an acceleration spike that
 * the chassis was never asked to produce.
 *
 * @param axMetersPerSecondSq forward acceleration, robot frame
 * @param ayMetersPerSecondSq left acceleration, robot frame
 * @param alphaRadiansPerSecondSq angular acceleration about the vertical axis
 */
public record ChassisAccel(
        double axMetersPerSecondSq, double ayMetersPerSecondSq, double alphaRadiansPerSecondSq) {}
