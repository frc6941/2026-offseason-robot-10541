package frc.robot.subsystems.FloorRoller;

import static frc.robot.RobotConstants.CANIVORE_CAN_BUS;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;

import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class FloorRollerConfig {
    private static final CANBus CANBUS = CANIVORE_CAN_BUS;

    private static final int FLOOR_ROLLER_ID = 20;

    public static final String FLOOR_ROLLER_NAME = "FloorRoller";

    private static final double FLOOR_ROLLER_GEAR_RATIO = 35.0 / 20.0;

    public static final SubsystemConfig FLOOR_ROLLER_CONFIG = SubsystemConfig.builder()
            .name(FLOOR_ROLLER_NAME)
            .mainBus(CANBUS)
            .mainId(FLOOR_ROLLER_ID)
            .motorInvertedValue(InvertedValue.Clockwise_Positive)
            .SensorToMechanismRatio(FLOOR_ROLLER_GEAR_RATIO)
            .build();

    @NTParameter(tableName = "Params/" + FLOOR_ROLLER_NAME)
    public static final class FloorRollerParams {
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kS = 0.0;

        public static final double feedRPS = 0.6;
        public static final double shootRPS = 1.0;
        public static final double idleRPS = 0.2;

        private FloorRollerParams() {}
    }

    private FloorRollerConfig() {}
}
