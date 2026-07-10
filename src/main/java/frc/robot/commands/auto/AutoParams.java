package frc.robot.commands.auto;

import com.pathplanner.lib.path.PathConstraints;
import lib.ntext.NTParameter;

/**
 * NetworkTables-tunable parameters for the autonomous routines. Scalars are exposed directly as NT
 * doubles; object-typed values (e.g. {@link PathConstraints}) are rebuilt from their live
 * components via the factory methods below so dashboard edits take effect without a redeploy.
 */
public class AutoParams {

    /** PID gains for the PathPlanner holonomic drive controller (read once in AutoBuilder). */
    @NTParameter(tableName = "Params/AutoBuilder")
    public static final class AutoBuilderParams {
        public static final double translationKP = 5.0;
        public static final double translationKI = 0.0;
        public static final double translationKD = 0.0;
        public static final double rotationKP = 5.0;
        public static final double rotationKI = 0.0;
        public static final double rotationKD = 0.0;
    }

    /** Tunable constants for the auto command factories in {@link AutoCommands}. */
    @NTParameter(tableName = "Params/AutoCommands")
    public static final class AutoCommandParams {
        // Path constraints: max linear velocity (m/s), linear accel (m/s^2), angular velocity and
        // angular accel (stored in deg/s and deg/s^2 for readability; converted to rad on build).
        public static final double preciseMaxVel = 4.0;
        public static final double preciseMaxAccel = 2.0;
        public static final double preciseMaxAngVelDeg = 360.0;
        public static final double preciseMaxAngAccelDeg = 540.0;

        public static final double intakeMediumMaxVel = 4.0;
        public static final double intakeMediumMaxAccel = 2.5;
        public static final double intakeMediumMaxAngVelDeg = 360.0;
        public static final double intakeMediumMaxAngAccelDeg = 540.0;

        public static final double transitMaxVel = 4.0;
        public static final double transitMaxAccel = 4.0;
        public static final double transitMaxAngVelDeg = 540.0;
        public static final double transitMaxAngAccelDeg = 720.0;

        // Timeouts / durations (seconds).
        public static final double autoDurationSeconds = 20.0;
        public static final double autoReturnTimeoutSeconds = 3.0;
        public static final double autoShootReadyTimeoutSeconds = 2.0;
        public static final double autoShootFeedSeconds = 3.0;
        public static final double autoMoveShotFeedSeconds = 2.5;

        // Goal end / through velocities (m/s).
        public static final double autoThroughVelocity = 2.0;
        public static final double autoIntakeThroughVelocity = 1.5;

        // Move-shot dynamic-pose PID gains.
        public static final double moveShotTranslationKP = 10.0;
        public static final double moveShotTranslationKD = 0.35;
        public static final double moveShotRotationKP = 8.0;
        public static final double moveShotRotationKD = 0.2;

        // Goal tolerances.
        public static final double autoTranslationToleranceMeters = 0.10;
        public static final double autoRotationToleranceDegrees = 3.0;

        // Number of samples used to build a mid-line wave sweep path. NT integers deserialize as
        // Long and break the generated wrapper, so this is kept as a double and cast on use.
        public static final double midWaveSampleCount = 9.0;
    }

    private AutoParams() {}

    // --- Object-typed factories, rebuilt live from the components above ---

    public static PathConstraints preciseConstraints() {
        return new PathConstraints(
                AutoCommandParamsNT.preciseMaxVel.getValue(),
                AutoCommandParamsNT.preciseMaxAccel.getValue(),
                Math.toRadians(AutoCommandParamsNT.preciseMaxAngVelDeg.getValue()),
                Math.toRadians(AutoCommandParamsNT.preciseMaxAngAccelDeg.getValue()));
    }

    public static PathConstraints intakeMediumConstraints() {
        return new PathConstraints(
                AutoCommandParamsNT.intakeMediumMaxVel.getValue(),
                AutoCommandParamsNT.intakeMediumMaxAccel.getValue(),
                Math.toRadians(AutoCommandParamsNT.intakeMediumMaxAngVelDeg.getValue()),
                Math.toRadians(AutoCommandParamsNT.intakeMediumMaxAngAccelDeg.getValue()));
    }

    public static PathConstraints transitConstraints() {
        return new PathConstraints(
                AutoCommandParamsNT.transitMaxVel.getValue(),
                AutoCommandParamsNT.transitMaxAccel.getValue(),
                Math.toRadians(AutoCommandParamsNT.transitMaxAngVelDeg.getValue()),
                Math.toRadians(AutoCommandParamsNT.transitMaxAngAccelDeg.getValue()));
    }
}
