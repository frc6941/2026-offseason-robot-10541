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

    // Shooter motors are numbered by physical layout:
    // left side top-to-bottom, then right side top-to-bottom.
    private static final int SHOOTER_LEFT_TOP_ID = 13;
    private static final int SHOOTER_LEFT_MIDDLE_ID = 14;
    private static final int SHOOTER_LEFT_BOTTOM_ID = 15;
    private static final int SHOOTER_RIGHT_TOP_ID = 16;
    private static final int SHOOTER_RIGHT_UPPER_MIDDLE_ID = 17;
    private static final int SHOOTER_RIGHT_LOWER_MIDDLE_ID = 18;
    private static final int SHOOTER_RIGHT_BOTTOM_ID = 19;
    private static final double SHOOTER_UPPER_GEAR_RATIO = 27.0 / 20.0;
    private static final double SHOOTER_LOWER_GEAR_RATIO = 35.0 / 20.0;
    private static final double HOOD_GEAR_RATIO = 5.0 * 156.0 / 22.0;

    public static final String SHOOTER_UPPER_NAME = "ShooterUpper";
    public static final String SHOOTER_LOWER_NAME = "ShooterLower";
    public static final String HOOD_NAME = "Hood";

    public static final Translation2d SHOOTER_TRANSLATION_FROM_ROBOT_CENTER =
            new Translation2d(0.0, 0.0);
    public static final Angle SHOOTER_FIRING_YAW_OFFSET = Degrees.of(0.0);
    public static final Angle HOOD_MIN_ANGLE = Degrees.of(0.0);
    public static final Angle HOOD_MAX_ANGLE = Degrees.of(20.0);
    public static final Angle HOOD_STOW_ANGLE = HOOD_MIN_ANGLE;
    public static final Angle HOOD_ANGLE_PER_ROTATION = Degrees.of(360.0);
    public static final Angle HOOD_ZERO_OFFSET = Degrees.of(0.0);

    public static final SubsystemConfig SHOOTER_UPPER_CONFIG = SubsystemConfig.builder()
            .name(SHOOTER_UPPER_NAME)
            .mainId(SHOOTER_LEFT_TOP_ID)
            .mainBus(CANBUS)
            .motorInvertedValue(InvertedValue.Clockwise_Positive)
            .SensorToMechanismRatio(SHOOTER_UPPER_GEAR_RATIO)
            .simConfig(SubsystemConfig.SimConfig.builder().gearRatio(SHOOTER_UPPER_GEAR_RATIO).build())
            .followers(new SubsystemConfig.FollowerConfig[] {
                    SubsystemConfig.FollowerConfig.builder()
                            .id(SHOOTER_LEFT_MIDDLE_ID)
                            .bus(CANBUS)
                            .build(),
                    SubsystemConfig.FollowerConfig.builder()
                            .id(SHOOTER_RIGHT_TOP_ID)
                            .bus(CANBUS)
                            .build(),
                    SubsystemConfig.FollowerConfig.builder()
                            .id(SHOOTER_RIGHT_UPPER_MIDDLE_ID)
                            .bus(CANBUS)
                            .build()
            })
            .build();

    public static final SubsystemConfig SHOOTER_LOWER_CONFIG = SubsystemConfig.builder()
            .name(SHOOTER_LOWER_NAME)
            .mainId(SHOOTER_RIGHT_LOWER_MIDDLE_ID)
            .mainBus(CANBUS)
            .motorInvertedValue(InvertedValue.Clockwise_Positive)
            .SensorToMechanismRatio(SHOOTER_LOWER_GEAR_RATIO)
            .simConfig(SubsystemConfig.SimConfig.builder().gearRatio(SHOOTER_LOWER_GEAR_RATIO).build())
            .followers(new SubsystemConfig.FollowerConfig[] {
                    SubsystemConfig.FollowerConfig.builder()
                            .id(SHOOTER_RIGHT_BOTTOM_ID)
                            .bus(CANBUS)
                            .build()
            })
            .build();

    public static final SubsystemConfig HOOD_CONFIG = SubsystemConfig.builder()
            .name(HOOD_NAME)
            .mainId(SHOOTER_LEFT_BOTTOM_ID)
            .mainBus(CANBUS)
            .motorInvertedValue(InvertedValue.Clockwise_Positive)
            .SensorToMechanismRatio(HOOD_GEAR_RATIO)
            .simConfig(
                    SubsystemConfig.SimConfig.builder()
                            .gearRatio(HOOD_GEAR_RATIO)
                            .build())
            .reverseSoftLimitDegrees(HOOD_MIN_ANGLE)
            .forwardSoftLimitDegrees(HOOD_MAX_ANGLE)
            .zeroOffset(HOOD_ZERO_OFFSET)
            .build();

    @NTParameter(tableName = "Params/" + SHOOTER_UPPER_NAME)
    public static final class ShooterUpperParams {
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kS = 0.0;

        public static final double shootRPS = 80.0;
        public static final double idleRPS = 20.0;
        public static final double velocityAtGoalToleranceRPS = 1.0;

        private ShooterUpperParams() {}
    }

    @NTParameter(tableName = "Params/" + SHOOTER_LOWER_NAME)
    public static final class ShooterLowerParams {
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kS = 0.0;

        public static final double shootRPS = 80.0;
        public static final double idleRPS = 20.0;
        public static final double velocityAtGoalToleranceRPS = 1.0;

        private ShooterLowerParams() {}
    }

    @NTParameter(tableName = "Params/" + HOOD_NAME)
    public static final class HoodParams {
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kS = 0.0;
        public static final double kG = 0.0;

        public static final double motionMagicVelRPS = 4.0;
        public static final double motionMagicAccelRPS2 = 16.0;
        public static final double motionMagicJerkRPS3 = 0.0;
        public static final double positionAtGoalToleranceDegrees = 0.5;

        private HoodParams() {}
    }

    private ShooterConfig() {}
}
