package frc.robot.subsystems.Hopper;

import static frc.robot.RobotConstants.ROBORIO_CAN_BUS;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class HopperConfig {
    private static final CANBus CANBUS = ROBORIO_CAN_BUS;

    private static final int HOPPER_L_ID = 20;
    private static final int HOPPER_R_ID = 21;

    public static final String HOPPER_NAME = "Hopper";

    private static final double HOPPER_GEAR_RATIO = 12.0 / 24.0;

    // Supply/stator caps for the 2 hopper Kraken X60s (an unset limit means UNLIMITED). Both motors
    // set their own limit — followers do NOT inherit the leader's. Supply is per-motor, so the pair
    // can pull up to 2x this from the battery. A hopper can jam, so keep the stator cap moderate.
    public static final double HOPPER_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
    public static final double HOPPER_STATOR_CURRENT_LIMIT_AMPS = 60.0;

    public static final SubsystemConfig HOPPER_CONFIG =
            SubsystemConfig.builder()
                    .name(HOPPER_NAME)
                    .mainBus(CANBUS)
                    .mainId(HOPPER_R_ID)
                    .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
                    .SensorToMechanismRatio(HOPPER_GEAR_RATIO)
                    .statorCurrentLimitAmps(HOPPER_STATOR_CURRENT_LIMIT_AMPS)
                    .supplyCurrentLimitAmps(HOPPER_SUPPLY_CURRENT_LIMIT_AMPS)
                    .followers(
                            new SubsystemConfig.FollowerConfig[] {
                                SubsystemConfig.FollowerConfig.builder()
                                        .id(HOPPER_L_ID)
                                        .bus(CANBUS)
                                        .opposeMain(MotorAlignmentValue.Opposed)
                                        .statorCurrentLimitAmps(HOPPER_STATOR_CURRENT_LIMIT_AMPS)
                                        .supplyCurrentLimitAmps(HOPPER_SUPPLY_CURRENT_LIMIT_AMPS)
                                        .build()
                            })
                    .build();

    @NTParameter(tableName = "Params/" + HOPPER_NAME)
    public static final class HopperParams {
        public static final double kP = 2;
        public static final double kI = 0.1;
        public static final double kD = 0.0;

        public static final double kV = 0.17;
        public static final double kA = 0.004;
        public static final double kS = 0.252;

        public static final double feedRPS = -0;
        public static final double shootRPS = 145;
        public static final double idleRPS = -0;
        public static final double outRPS = -50;

        private HopperParams() {}
    }

    private HopperConfig() {}
}
