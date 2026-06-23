package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class RobotConstants {
    // CAN
    public static final String ROBORIO_CAN_BUS_NAME = "rio";
    public static final String CANIVORE_CAN_BUS_NAME = "6941Canivore0";
    public static final CANBus CANIVORE_CAN_BUS = new CANBus(CANIVORE_CAN_BUS_NAME);


    // CAN_ID
    public static final int PIGEON_ID = 14;

    
    // Alliance flip
    public static boolean disableHAL = false;

    // Robot Periodic
    public static final double LOOPER_DT = 0.02;

    

    private RobotConstants() {}
}
