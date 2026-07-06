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
    private static final double SHOOTER_TOP_CONTROL_GEAR_RATIO = 5.0 * 156.0 / 22.0;

    public static final String SHOOTER_UPPER_NAME = "ShooterUpper";
    public static final String SHOOTER_LOWER_NAME = "ShooterLower";
    public static final String SHOOTER_TOP_CONTROL_NAME = "ShooterTopControl";

    public static final Translation2d SHOOTER_TRANSLATION_FROM_ROBOT_CENTER =
            new Translation2d(0.0, 0.0);
    public static final Angle SHOOTER_FIRING_YAW_OFFSET = Degrees.of(0.0);
    public static final Angle SHOOTER_TOP_CONTROL_MIN_ANGLE = Degrees.of(0.0);
    public static final Angle SHOOTER_TOP_CONTROL_MAX_ANGLE = Degrees.of(20.0);
    public static final Angle SHOOTER_TOP_CONTROL_STOW_ANGLE = SHOOTER_TOP_CONTROL_MIN_ANGLE;
    public static final Angle SHOOTER_TOP_CONTROL_ANGLE_PER_ROTATION =
            Degrees.of(360.0 / SHOOTER_TOP_CONTROL_GEAR_RATIO);
    public static final Angle SHOOTER_TOP_CONTROL_MAX_MOTOR_TRAVEL =
            Degrees.of(
                    SHOOTER_TOP_CONTROL_MAX_ANGLE.in(Degrees)
                            * SHOOTER_TOP_CONTROL_GEAR_RATIO);
    public static final Angle SHOOTER_TOP_CONTROL_ZERO_OFFSET = Degrees.of(0.0);

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

    public static final SubsystemConfig SHOOTER_TOP_CONTROL_CONFIG = SubsystemConfig.builder()
            .name(SHOOTER_TOP_CONTROL_NAME)
            .mainId(SHOOTER_LEFT_BOTTOM_ID)
            .mainBus(CANBUS)
            .motorInvertedValue(InvertedValue.Clockwise_Positive)
            .SensorToMechanismRatio(SHOOTER_TOP_CONTROL_GEAR_RATIO)
            .simConfig(
                    SubsystemConfig.SimConfig.builder()
                            .gearRatio(SHOOTER_TOP_CONTROL_GEAR_RATIO)
                            .build())
            .reverseSoftLimitDegrees(SHOOTER_TOP_CONTROL_MIN_ANGLE)
            .forwardSoftLimitDegrees(SHOOTER_TOP_CONTROL_MAX_MOTOR_TRAVEL)
            .zeroOffset(SHOOTER_TOP_CONTROL_ZERO_OFFSET)
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

    @NTParameter(tableName = "Params/" + SHOOTER_TOP_CONTROL_NAME)
    public static final class ShooterTopControlParams {
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

        private ShooterTopControlParams() {}
    }

    private ShooterConfig() {}
}
