package frc.robot.subsystems.Intaker;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.RobotConstants.ROBORIO_CAN_BUS;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class IntakerConfig {

    private final static CANBus CANBUS = ROBORIO_CAN_BUS;

    private final static int INTAKER_PIVOT_ID = 22;
    private final static int INTAKER_ROLLER_ID = 23;

    public final static String INTAKER_ROLLER_NAME = "IntakerRoller";
    public final static String INTAKER_PIVOT_NAME = "IntakerPivot";

    private final static double INTAKER_ROLLER_GEAR_RATIO = 20.0 / 35.0;
    private final static double INTAKER_PIVOT_GEAR_RATIO = 1.0 / 12 * 20 / 36 * 20 / 36 * 15 / 36;

    // Mechanism degrees per ONE mechanism rotation. The gear reduction is handled by Phoenix's
    // SensorToMechanismRatio (see INTAKER_PIVOT_CONFIG), NOT here — a bare 360, or the ratio gets
    // applied twice and the pivot barely moves. See PositionMotorSubsystem doc ("Pivot: 360").
    public final static Angle INTAKER_ANGLE_PER_ROTATION = Degrees.of(360.0);
    public final static Angle INTAKER_PIVOT_MIN_ANGLE = Degrees.of(0.0);
    public final static Angle INTAKER_PIVOT_MAX_ANGLE = Degrees.of(135.0);

    // The real intake-pivot angle when the mechanism is resting against its zero hard stop.
    // Measure on the robot and update this value so getCurrPos() reports the true mechanism angle after homing.
    public final static Angle INTAKER_PIVOT_ZERO_OFFSET = Degrees.of(0.0);
    // Placeholder template values for homing. These MUST be verified on the real robot.
    // Sign must drive the intake pivot toward the zero hard stop.
    public final static double INTAKER_PIVOT_ZEROING_VOLTAGE = -1.0;
    // Must be above free-run current and below breaker / unsafe stall current.
    public final static double INTAKER_PIVOT_ZEROING_CURRENT_LIMIT_AMPS = 20.0;
    public final static int INTAKER_PIVOT_ZEROING_FILTER_SIZE = 5;

    public enum IntakeMode{
        INTAKING,           // Roller: Intake, Pivot: Extended
        RETRACTED,          // Roller: Stop,   Pivot: Retracted
        FEEDING,            // Roller: Intake, Pivot: Extended
        EXTENDED_REVERSE,   // Roller: Outtake,Pivot: Extended
        RETRACTED_FEEDING,  // Roller: Intake, Pivot: Swing
        EXTENDED_IDLE       // Roller: Stop,   Pivot: Extended
    }

    public final static SubsystemConfig INTAKER_ROLLER_CONFIG = SubsystemConfig.builder()
            .name(INTAKER_ROLLER_NAME)
            .mainBus(CANBUS)
            .mainId(INTAKER_ROLLER_ID)
            .motorInvertedValue(InvertedValue.Clockwise_Positive)
            .SensorToMechanismRatio(INTAKER_ROLLER_GEAR_RATIO)
            .build();

    public final static SubsystemConfig INTAKER_PIVOT_CONFIG = SubsystemConfig.builder()
            .name(INTAKER_PIVOT_NAME)
            .mainBus(CANBUS)
            .mainId(INTAKER_PIVOT_ID)
            .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
            // Phoenix wants ROTOR rotations per MECHANISM rotation (the reduction, ~93), which is
            // 1/INTAKER_PIVOT_GEAR_RATIO — not the ratio itself (~0.0107, the reciprocal).
            .SensorToMechanismRatio(1.0 / INTAKER_PIVOT_GEAR_RATIO)
            // Normal-operation protection range. zeroCommand() disables these temporarily during homing.
            .reverseSoftLimitDegrees(INTAKER_PIVOT_MIN_ANGLE)
            .forwardSoftLimitDegrees(INTAKER_PIVOT_MAX_ANGLE)
            .zeroOffset(INTAKER_PIVOT_ZERO_OFFSET)
            .zeroingConfig(SubsystemConfig.ZeroingConfig.builder()
                    .zeroingVoltage(INTAKER_PIVOT_ZEROING_VOLTAGE)
                    .zeroingCurrentLimit(INTAKER_PIVOT_ZEROING_CURRENT_LIMIT_AMPS)
                    .zeroingFilterSize(INTAKER_PIVOT_ZEROING_FILTER_SIZE)
                    .build())
            .build();



    @NTParameter(tableName = "Params/"+INTAKER_ROLLER_NAME)
    public final static class IntakerRollerParams{
        public final static double kP = 0.0;
        public final static double kI = 0.0;
        public final static double kD = 0.0;

        public final static double kV = 0.0;
        public final static double kA = 0.0;
        public final static double kS = 0.0;

        // NT integers come back from NetworkTables as Long, which breaks the generated
        // Integer wrapper on refresh() — keep all tunable NT numerics as double.
        public final static double intakeRPS = 20.0;
        public final static double outtakeRPS = -15.0;
    }

    @NTParameter(tableName = "Params/"+INTAKER_PIVOT_NAME)
    public final static class IntakerPivotParams{
        public final static double kP = 0.0;
        public final static double kI = 0.0;
        public final static double kD = 0.0;

        public final static double kV = 0.0;
        public final static double kA = 0.0;
        public final static double kS = 0.0;

        public static final double motionMagicVelRPS = 1000.0;
        public static final double motionMagicAccelRPS2 = 150.0;
        public static final double motionMagicJerkRPS3 = 0.0;

        public static final double deployPosAngle = 0.0;
        public static final double retractPosAngle = 135.0;
        public static final double feedPosAngle = 35.0;
        public static final double retractedfeedPosAngle = 75.0;

    }

}
