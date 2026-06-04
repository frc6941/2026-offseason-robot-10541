package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class RobotConstants {
    public static final String ROBORIO_CAN_BUS_NAME = "rio";
    public static final String CANIVORE_CAN_BUS_NAME = "6941Canivore0";
    public static final CANBus CANIVORE_CAN_BUS = new CANBus(CANIVORE_CAN_BUS_NAME);
    public static boolean disableHAL = false;
    public static final double LOOPER_DT = 0.02;
    public static final int PIGEON_ID = 14;

    private RobotConstants() {}
}
