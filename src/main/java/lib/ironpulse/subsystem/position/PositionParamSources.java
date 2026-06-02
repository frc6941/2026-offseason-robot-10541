package lib.ironpulse.subsystem.position;

// hack to hook NTParameterProcessor to generate ParamSources for uses in subsystems
/**
 * REMEMBER to update {@link lib.ntext.NTParameterProcessor} when adding new fields to ParamSources
 */
public interface PositionParamSources {
    double kP();

    double kI();

    double kD();

    default double kA() {
        return 0.0;
    }

    default double kV() {
        return 0.0;
    }

    default double kS() {
        return 0.0;
    }

    default double kG() {
        return 0.0;
    }

    default double motionMagicVelRPS() {
        return 0.0;
    }

    default double motionMagicAccelRPS2() {
        return 0.0;
    }

    default double motionMagicJerkRPS3() {
        return 0.0;
    }

    default double positionAtGoalToleranceDegrees() {
        return 1.0;
    }

    default double positionAtGoalToleranceMeters() {
        return 0.005;
    }

    default boolean isBrake() {
        return true;
    }

    /* hook to ParamsNT.isAnyChanged() */
    default boolean hasChanged() {
        return false;
    }
}
