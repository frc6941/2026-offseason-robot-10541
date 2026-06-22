package frc.robot.subsystems.Intaker;

import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.RobotConstants;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class IntakerConfig {
    public final static String INTAKER_ROLLER_NAME = "Intaker Roller";
    private final static int INTAKER_ROLLER_ID = 10;
    private final static double INTAKER_ROLLER_GEAR_RATIO = 0.0;

    public final static SubsystemConfig INTAKER_ROLLER_CONFIG = SubsystemConfig.builder()
            .name(INTAKER_ROLLER_NAME)
            .mainBus(RobotConstants.CANIVORE_CAN_BUS)
            .mainId(INTAKER_ROLLER_ID)
            .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
            .SensorToMechanismRatio(INTAKER_ROLLER_GEAR_RATIO)
            .build();


    @NTParameter(tableName = "Params/"+INTAKER_ROLLER_NAME)
    public final static class IntakerParams{
        public final static double kP = 0.0;
        public final static double kI = 0.0;
        public final static double kD = 0.0;

        public final static double kV = 0.0;
        public final static double kA = 0.0;
        public final static double kS = 0.0;
    }



}
