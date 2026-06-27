package frc.robot.subsystems.Hopper;

import static frc.robot.RobotConstants.CANIVORE_CAN_BUS;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;

import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class HopperConfig {
    private static final CANBus CANBUS = CANIVORE_CAN_BUS;

    private static final int HOPPER_ID = 20;

    public static final String HOPPER_NAME = "Hopper";

    private static final double HOPPER_GEAR_RATIO = 35.0 / 20.0;

    public static final SubsystemConfig HOPPER_CONFIG = SubsystemConfig.builder()
            .name(HOPPER_NAME)
            .mainBus(CANBUS)
            .mainId(HOPPER_ID)
            .motorInvertedValue(InvertedValue.Clockwise_Positive)
            .SensorToMechanismRatio(HOPPER_GEAR_RATIO)
            .build();

    @NTParameter(tableName = "Params/" + HOPPER_NAME)
    public static final class HopperParams {
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kS = 0.0;

        public static final double feedRPS = 0.6;
        public static final double shootRPS = 1.0;
        public static final double idleRPS = 0.2;

        private HopperParams() {}
    }

    private HopperConfig() {}
}
