package frc.robot.subsystems.Intaker;

import static frc.robot.RobotConstants.CANIVORE_CAN_BUS;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;

import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class IntakerConfig {

    private final static CANBus CANBUS = CANIVORE_CAN_BUS;

    private final static int INTAKER_ROLLER_ID = 10;
    private final static int INTAKER_EXTENSION_ID = 11;

    public final static String INTAKER_ROLLER_NAME = "IntakerRoller";
    public final static String INTAKER_EXTENSION_NAME = "IntakerExtension";

    private final static double INTAKER_ROLLER_GEAR_RATIO = 35.0 / 20.0;
    private final static double INTAKER_EXTENSION_GEAR_RATIO = 48.0 / 5.0;

    public final static SubsystemConfig INTAKER_ROLLER_CONFIG = SubsystemConfig.builder()
            .name(INTAKER_ROLLER_NAME)
            .mainBus(CANBUS)
            .mainId(INTAKER_ROLLER_ID)
            .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
            .SensorToMechanismRatio(INTAKER_ROLLER_GEAR_RATIO)
            .build();

    public final static SubsystemConfig INTAKER_EXTENSION_CONFIG = SubsystemConfig.builder()
            .name(INTAKER_EXTENSION_NAME)
            .mainBus(CANBUS)
            .mainId(INTAKER_EXTENSION_ID)
            .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
            .SensorToMechanismRatio(INTAKER_EXTENSION_GEAR_RATIO)
            .build();



    @NTParameter(tableName = "Params/"+INTAKER_ROLLER_NAME)
    public final static class IntakerRollerParams{
        public final static double kP = 0.0;
        public final static double kI = 0.0;
        public final static double kD = 0.0;

        public final static double kV = 0.0;
        public final static double kA = 0.0;
        public final static double kS = 0.0;
    }

    @NTParameter(tableName = "Params/"+INTAKER_EXTENSION_NAME)
    public final static class IntakerExtensionParams{
        public final static double kP = 0.0;
        public final static double kI = 0.0;
        public final static double kD = 0.0;

        public final static double kV = 0.0;
        public final static double kA = 0.0;
        public final static double kS = 0.0;

    }


}
