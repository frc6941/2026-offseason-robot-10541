package frc.robot.subsystems.Hopper;

import static frc.robot.RobotConstants.ROBORIO_CAN_BUS;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ironpulse.subsystem.velocity.VelocityParamSources;

public class HopperConfig {
    private static final CANBus CANBUS = ROBORIO_CAN_BUS;

    private static final int HOPPER_L_ID = 20;
    private static final int HOPPER_R_ID = 21;

    public static final String HOPPER_NAME = "Hopper";

    private static final double HOPPER_GEAR_RATIO = 12.0 / 24.0;

    public static final SubsystemConfig HOPPER_CONFIG =
            SubsystemConfig.builder()
                    .name(HOPPER_NAME)
                    .mainBus(CANBUS)
                    .mainId(HOPPER_R_ID)
                    .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
                    .SensorToMechanismRatio(HOPPER_GEAR_RATIO)
                    .followers(
                            new SubsystemConfig.FollowerConfig[] {
                                SubsystemConfig.FollowerConfig.builder()
                                        .id(HOPPER_L_ID)
                                        .bus(CANBUS)
                                        .opposeMain(MotorAlignmentValue.Opposed)
                                        .build()
                            })
                    .build();

    public static final class HopperParams {
        public static final double kP = 2;
        public static final double kI = 0.01;
        public static final double kD = 0.0;

        public static final double kV = 0.15;
        public static final double kA = 0.0;
        public static final double kS = 3.4;

        public static final double feedRPS = 20;
        public static final double shootRPS = 40;
        public static final double idleRPS = 0;

        public static VelocityParamSources asVelocityParamSources() {
            return new VelocityParamSources() {
                public double kP() {
                    return kP;
                }

                public double kI() {
                    return kI;
                }

                public double kD() {
                    return kD;
                }

                public double kV() {
                    return kV;
                }

                public double kA() {
                    return kA;
                }

                public double kS() {
                    return kS;
                }
            };
        }

        private HopperParams() {}
    }

    private HopperConfig() {}
}
