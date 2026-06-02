package frc.robot;

import com.ctre.phoenix6.CANBus;

/** Stub — replace with real constants as subsystems are added. */
public final class RobotConstants {
    public static final String ROBORIO_CAN_BUS_NAME = "rio";
    public static final String CANIVORE_CAN_BUS_NAME = "6941Canivore0";
    public static final CANBus CANIVORE_CAN_BUS = new CANBus(CANIVORE_CAN_BUS_NAME);
    public static boolean disableHAL = false;

    private RobotConstants() {}
}
