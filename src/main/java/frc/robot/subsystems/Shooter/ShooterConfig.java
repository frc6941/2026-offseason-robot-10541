package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.RobotConstants.CANIVORE_CAN_BUS;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class ShooterConfig {
    private static final CANBus CANBUS = CANIVORE_CAN_BUS;

    private static final int SHOOTER_MAIN_ID = 19;
    private static final int SHOOTER_FOLLOWER_ID = 20;

    public static final String SHOOTER_NAME = "Shooter";

    public static final Translation2d SHOOTER_TRANSLATION_FROM_ROBOT_CENTER =
            new Translation2d(0.0, 0.0);
    public static final Angle SHOOTER_FIRING_YAW_OFFSET = Degrees.of(0.0);

    public static final SubsystemConfig SHOOTER_CONFIG = SubsystemConfig.builder()
            .name(SHOOTER_NAME)
            .mainId(SHOOTER_MAIN_ID)
            .mainBus(CANBUS)
            .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
            .followers(new SubsystemConfig.FollowerConfig[] {
                    SubsystemConfig.FollowerConfig.builder()
                            .id(SHOOTER_FOLLOWER_ID)
                            .bus(CANBUS)
                            .build()
            })
            .build();

    @NTParameter(tableName = "Params/" + SHOOTER_NAME)
    public static final class ShooterParams {
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kS = 0.0;

        public static final double shootRPS = 80.0;
        public static final double idleRPS = 0.0;
        public static final double velocityAtGoalToleranceRPS = 1.0;

        private ShooterParams() {}
    }

    private ShooterConfig() {}
}
