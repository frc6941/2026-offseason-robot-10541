package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.RobotConstants.CANIVORE_CAN_BUS;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class HoodConfig {
    private static final CANBus CANBUS = CANIVORE_CAN_BUS;

    private static final int HOOD_ID = 10;
    private static final double HOOD_GEAR_RATIO = 1.0;

    public static final String HOOD_NAME = "Hood";

    public static final Angle HOOD_MIN_ANGLE = Degrees.of(0.0);
    public static final Angle HOOD_MAX_ANGLE = Degrees.of(30.0);
    public static final Angle HOOD_STOW_ANGLE = HOOD_MIN_ANGLE;
    public static final Angle HOOD_ANGLE_PER_ROTATION = Degrees.of(360.0 / HOOD_GEAR_RATIO);

    public static final SubsystemConfig HOOD_CONFIG = SubsystemConfig.builder()
            .name(HOOD_NAME)
            .mainId(HOOD_ID)
            .mainBus(CANBUS)
            .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
            .SensorToMechanismRatio(HOOD_GEAR_RATIO)
            .build();

    @NTParameter(tableName = "Params/" + HOOD_NAME)
    public static final class HoodParams {
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kS = 0.0;
        public static final double kG = 0.0;

        public static final double motionMagicVelRPS = 1000.0;
        public static final double motionMagicAccelRPS2 = 150.0;
        public static final double motionMagicJerkRPS3 = 0.0;
        public static final double positionAtGoalToleranceDegrees = 1.0;

        private HoodParams() {}
    }

    private HoodConfig() {}
}
