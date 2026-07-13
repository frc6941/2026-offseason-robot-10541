package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.RobotConstants.ROBORIO_CAN_BUS;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import lib.ironpulse.subsystem.SubsystemConfig;
<<<<<<< HEAD
=======
import lib.ironpulse.subsystem.position.PositionParamSources;
import lib.ironpulse.subsystem.velocity.VelocityParamSources;
>>>>>>> newauto
import lib.ntext.NTParameter;

public class ShooterConfig {
    private static final CANBus CANBUS = ROBORIO_CAN_BUS;

    // Shooter motors are numbered by physical layout:
    // left side top-to-bottom, then right side top-to-bottom.
    private static final int SHOOTER_L1_ID = 13;
    private static final int SHOOTER_L2_ID = 14;
    private static final int SHOOTER_R1_ID = 15;
    private static final int SHOOTER_R2_ID = 16;
    private static final int SHOOTER_R3_ID = 17;
    private static final int SHOOTER_R4_ID = 18;
    private static final int HOOD_ID = 19;

    private static final double SHOOTER_DRUM_GEAR_RATIO = 20.0 / 27.0;
    private static final double SHOOTER_FEED_GEAR_RATIO = 20.0 / 35.0;
    private static final double HOOD_GEAR_RATIO = 8.0 / 50.0 * 10.0 / 156.0;

    public static final String SHOOTER_DRUM_NAME = "ShooterDrum";
    public static final String SHOOTER_FEED_NAME = "ShooterFeed";
    public static final String HOOD_NAME = "Hood";

    public static final Translation2d SHOOTER_TRANSLATION_FROM_ROBOT_ORIGIN =
            new Translation2d(0.0, 0.0); // 257.9 mm
    public static final Angle SHOOTER_FIRING_YAW_OFFSET = Degrees.of(0.0);

    public static final Angle HOOD_MIN_ANGLE = Degrees.of(0.0);
    // Must equal the hood's true mechanical travel. ShotCalculator converts physical launch pitch
    // to this mechanism angle and clamps the result to this range.
    public static final Angle HOOD_MAX_ANGLE = Degrees.of(40.0);
    public static final Angle HOOD_STOW_ANGLE = HOOD_MIN_ANGLE;
    // Mechanism degrees per ONE mechanism rotation. The gear reduction is handled by Phoenix's
    // SensorToMechanismRatio (below), NOT here — so this must be a bare 360, or the ratio gets
    // applied twice and the hood barely moves. See PositionMotorSubsystem doc ("Pivot: 360").
    public static final Angle HOOD_ANGLE_PER_ROTATION = Degrees.of(360.0);
    public static final Angle HOOD_ZERO_OFFSET = Degrees.of(10.0);

    public static final SubsystemConfig SHOOTER_DRUM_CONFIG =
            SubsystemConfig.builder()
                    .name(SHOOTER_DRUM_NAME)
                    .mainId(SHOOTER_L1_ID)
                    .mainBus(CANBUS)
                    .motorInvertedValue(InvertedValue.Clockwise_Positive)
                    .SensorToMechanismRatio(SHOOTER_DRUM_GEAR_RATIO)
                    .simConfig(
                            SubsystemConfig.SimConfig.builder()
                                    .gearRatio(SHOOTER_DRUM_GEAR_RATIO)
                                    .build())
                    .followers(
                            new SubsystemConfig.FollowerConfig[] {
                                SubsystemConfig.FollowerConfig.builder()
                                        .id(SHOOTER_L2_ID)
                                        .bus(CANBUS)
                                        .build(),
                                SubsystemConfig.FollowerConfig.builder()
                                        .id(SHOOTER_R1_ID)
                                        .bus(CANBUS)
                                        .opposeMain(MotorAlignmentValue.Opposed)
                                        .build(),
                                SubsystemConfig.FollowerConfig.builder()
                                        .id(SHOOTER_R2_ID)
                                        .bus(CANBUS)
                                        .opposeMain(MotorAlignmentValue.Opposed)
                                        .build()
                            })
                    .build();

    public static final SubsystemConfig SHOOTER_FEED_CONFIG =
            SubsystemConfig.builder()
                    .name(SHOOTER_FEED_NAME)
                    .mainId(SHOOTER_R3_ID)
                    .mainBus(CANBUS)
                    .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
                    .SensorToMechanismRatio(SHOOTER_FEED_GEAR_RATIO)
                    .simConfig(
                            SubsystemConfig.SimConfig.builder()
                                    .gearRatio(SHOOTER_FEED_GEAR_RATIO)
                                    .build())
                    .followers(
                            new SubsystemConfig.FollowerConfig[] {
                                SubsystemConfig.FollowerConfig.builder()
                                        .id(SHOOTER_R4_ID)
                                        .bus(CANBUS)
                                        .build()
                            })
                    .build();

    public static final SubsystemConfig HOOD_CONFIG =
            SubsystemConfig.builder()
                    .name(HOOD_NAME)
                    .mainId(HOOD_ID)
                    .mainBus(CANBUS)
                    .motorInvertedValue(InvertedValue.Clockwise_Positive)
                    // Phoenix wants ROTOR rotations per MECHANISM rotation (the reduction, ~44.3),
                    // which is
                    // 1/HOOD_GEAR_RATIO — not HOOD_GEAR_RATIO (~0.023, the reciprocal).
                    .SensorToMechanismRatio(1.0 / HOOD_GEAR_RATIO)
                    .simConfig(
                            SubsystemConfig.SimConfig.builder()
                                    .gearRatio(1.0 / HOOD_GEAR_RATIO)
                                    .build())
                    .reverseSoftLimitDegrees(HOOD_MIN_ANGLE)
                    .forwardSoftLimitDegrees(HOOD_MAX_ANGLE)
                    .zeroOffset(HOOD_ZERO_OFFSET)
                    .build();

    @NTParameter(tableName = "Params/" + SHOOTER_DRUM_NAME)
    public static final class ShooterUpperParams {
        public static final double kP = 0.6;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kV = 0.09;
        public static final double kA = 0.01;
        public static final double kS = 0.01;

        public static final double shootRPS = 80.0;
        public static final double idleRPS = 5.0;
        public static final double velocityAtGoalToleranceRPS = 1.0;

        // Bench-test setpoint: drum (upper) RPS for the isolated spin-up test binding.
        public static final double testRPS = 40.0;

        private ShooterUpperParams() {}
    }

    @NTParameter(tableName = "Params/" + SHOOTER_FEED_NAME)
    public static final class ShooterLowerParams {
        public static final double kP = 0.3;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kV = 0.08;
        public static final double kA = 0.01;
        public static final double kS = 0.0;

        public static final double shootRPS = 80.0;
        public static final double idleRPS = 0.0;
        public static final double velocityAtGoalToleranceRPS = 1.0;

        private ShooterLowerParams() {}
    }

    @NTParameter(tableName = "Params/" + HOOD_NAME)
    public static final class HoodParams {
        public static final double kP = 100.0;
        public static final double kI = 0.0;
        public static final double kD = 0.01;

        public static final double kV = 6.0;
        public static final double kA = 0.1;
        public static final double kS = 0.18;
        public static final double kG = 0.18;

        public static final double motionMagicVelRPS = 1.0;
        public static final double motionMagicAccelRPS2 = 4.0;
        public static final double motionMagicJerkRPS3 = 0.0;
        public static final double positionAtGoalToleranceDegrees = 0.5;

        // Bench-test setpoint: target hood angle (deg) for the isolated rotate-to-angle test
        // binding.
        // Clamped to [HOOD_MIN_ANGLE, HOOD_MAX_ANGLE] before use.
        public static final double testAngleDeg = 10.0;

        private HoodParams() {}
    }

    private ShooterConfig() {}
}
