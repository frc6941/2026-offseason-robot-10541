package frc.robot.subsystems.Intaker;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.RobotConstants.ROBORIO_CAN_BUS;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import lib.ironpulse.subsystem.SubsystemConfig;
import lib.ntext.NTParameter;

public class IntakerConfig {

    private static final CANBus CANBUS = ROBORIO_CAN_BUS;

    private static final int INTAKER_PIVOT_ID = 22;
    private static final int INTAKER_ROLLER_ID = 23;

    public static final String INTAKER_ROLLER_NAME = "IntakerRoller";
    public static final String INTAKER_PIVOT_NAME = "IntakerPivot";

    private static final double INTAKER_ROLLER_GEAR_RATIO = 35.0 / 20.0;

    // Caps for the intake-roller Kraken X60 (an unset limit means UNLIMITED). The roller stalls on
    // game-piece contact, so the stator cap limits stall heat; bench-tune for grip vs. draw.
    public static final double INTAKER_ROLLER_STATOR_CURRENT_LIMIT_AMPS = 60.0;
    public static final double INTAKER_ROLLER_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
    private static final double INTAKER_PIVOT_GEAR_RATIO = 1.0 / 12 * 20 / 36 * 20 / 36 * 15 / 36;

    // Mechanism degrees per ONE mechanism rotation. The gear reduction is handled by Phoenix's
    // SensorToMechanismRatio (see INTAKER_PIVOT_CONFIG), NOT here — a bare 360, or the ratio gets
    // applied twice and the pivot barely moves. See PositionMotorSubsystem doc ("Pivot: 360").
    public static final Angle INTAKER_ANGLE_PER_ROTATION = Degrees.of(360.0);
    public static final Angle INTAKER_PIVOT_MIN_ANGLE = Degrees.of(0.0);
    public static final Angle INTAKER_PIVOT_MAX_ANGLE = Degrees.of(135.0);

    // The real intake-pivot angle when the mechanism is resting against its zero hard stop.
    // Measure on the robot and update this value so getCurrPos() reports the true mechanism angle
    // after homing.
    public static final Angle INTAKER_PIVOT_ZERO_OFFSET = Degrees.of(0.0);
    public static final double INTAKER_PIVOT_SHOOT_RAISE_SPEED_DEGREES_PER_SECOND = 64.0;
    public static final double INTAKER_PIVOT_STATOR_CURRENT_LIMIT_AMPS = 50.0;
    public static final double INTAKER_PIVOT_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
    // Placeholder template values for homing. These MUST be verified on the real robot.
    // Sign must drive the intake pivot toward the zero hard stop. Magnitude sets descent speed
    // through the ~93:1 reduction — was -2 (a slow creep that delayed the start of auto). Raised to
    // home faster so it finishes in the ball-free zone near the start pose, but kept below the -8
    // that slams the hard stop hard; bench-tune. Pairs with the overlap in AutoRoutines, which runs
    // homing concurrently with the opening transit but gates collecting on isPivotZeroed().
    public static final double INTAKER_PIVOT_ZEROING_VOLTAGE = -5;
    // Safety cap on the homing descent: if the hard stop is never detected (mechanism hung, motor
    // unplugged), stop pushing after this long instead of driving into the stop forever. On timeout
    // the pivot is left NOT zeroed, so auto declines to intake rather than trusting a false zero.
    public static final double INTAKER_PIVOT_ZEROING_TIMEOUT_SECONDS = 2.0;
    // Must be above free-run current and below breaker / unsafe stall current.
    public static final double INTAKER_PIVOT_ZEROING_CURRENT_LIMIT_AMPS = 40.0;
    public static final int INTAKER_PIVOT_ZEROING_FILTER_SIZE = 3;

    public enum IntakeMode {
        INTAKING, // Roller: Intake, Pivot: Extended
        MAX_INTAKING, // Roller: Max intake, Pivot: Extended
        RETRACTED, // Roller: Stop,   Pivot: Retracted
        FEEDING, // Roller: Intake, Pivot: Extended
        EXTENDED_REVERSE, // Roller: Outtake,Pivot: Extended
        RETRACTED_FEEDING, // Roller: Intake, Pivot: Swing
        EXTENDED_IDLE, // Roller: Stop,   Pivot: Extended
        DEPOT
    }

    public static final SubsystemConfig INTAKER_ROLLER_CONFIG =
            SubsystemConfig.builder()
                    .name(INTAKER_ROLLER_NAME)
                    .mainBus(CANBUS)
                    .mainId(INTAKER_ROLLER_ID)
                    .motorInvertedValue(InvertedValue.Clockwise_Positive)
                    .SensorToMechanismRatio(INTAKER_ROLLER_GEAR_RATIO)
                    .statorCurrentLimitAmps(INTAKER_ROLLER_STATOR_CURRENT_LIMIT_AMPS)
                    .supplyCurrentLimitAmps(INTAKER_ROLLER_SUPPLY_CURRENT_LIMIT_AMPS)
                    .build();

    public static final SubsystemConfig INTAKER_PIVOT_CONFIG =
            SubsystemConfig.builder()
                    .name(INTAKER_PIVOT_NAME)
                    .mainBus(CANBUS)
                    .mainId(INTAKER_PIVOT_ID)
                    .motorInvertedValue(InvertedValue.CounterClockwise_Positive)
                    // Phoenix wants ROTOR rotations per MECHANISM rotation (the reduction, ~93),
                    // which is
                    // 1/INTAKER_PIVOT_GEAR_RATIO — not the ratio itself (~0.0107, the reciprocal).
                    .SensorToMechanismRatio(1.0 / INTAKER_PIVOT_GEAR_RATIO)
                    .statorCurrentLimitAmps(INTAKER_PIVOT_STATOR_CURRENT_LIMIT_AMPS)
                    .supplyCurrentLimitAmps(INTAKER_PIVOT_SUPPLY_CURRENT_LIMIT_AMPS)
                    // Arm-type gravity: torque needed to hold position varies with cos(angle), so
                    // Slot0 kG is applied as an arm-cosine feedforward (not a constant elevator
                    // term).
                    .gravityType(GravityTypeValue.Arm_Cosine)
                    // Normal-operation protection range. zeroCommand() disables these temporarily
                    // during homing.
                    .reverseSoftLimitDegrees(INTAKER_PIVOT_MIN_ANGLE)
                    .forwardSoftLimitDegrees(INTAKER_PIVOT_MAX_ANGLE)
                    .zeroOffset(INTAKER_PIVOT_ZERO_OFFSET)
                    .zeroingConfig(
                            SubsystemConfig.ZeroingConfig.builder()
                                    .zeroingVoltage(INTAKER_PIVOT_ZEROING_VOLTAGE)
                                    .zeroingCurrentLimit(INTAKER_PIVOT_ZEROING_CURRENT_LIMIT_AMPS)
                                    .zeroingFilterSize(INTAKER_PIVOT_ZEROING_FILTER_SIZE)
                                    .build())
                    .build();

    @NTParameter(tableName = "Params/" + INTAKER_ROLLER_NAME)
    public static final class IntakerRollerParams {
        public static final double kP = 1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kV = 0.9;
        public static final double kA = 0.016835;
        public static final double kS = 0.17085;

        public static final double intakeRPS = 50.0;
        public static final double intakeRPSmax = 70.0;

        public static final double outtakeRPS = -50.0;

        private IntakerRollerParams() {}
    }

    @NTParameter(tableName = "Params/" + INTAKER_PIVOT_NAME)
    public static final class IntakerPivotParams {
        public static final double kP = 120.0;
        public static final double kI = 0.0;
        public static final double kD = 0.010;

        public static final double kV = 6.0;
        public static final double kA = 0.1;
        public static final double kS = 0.20;
        // Gravity feedforward: arm-cosine scaling applied by Phoenix. Start conservative (0.25V
        // at horizontal); tune upward if the pivot struggles to hold/reach raised angles (20°+).
        public static final double kG = 0.5;

        public static final double motionMagicVelRPS = 10.0;
        public static final double motionMagicAccelRPS2 = 20.0;
        public static final double motionMagicJerkRPS3 = 0.0;

        public static final double deployPosAngle = 10;
        public static final double retractPosAngle = 135.0;
        public static final double feedPosAngle = 35.0;
        public static final double depotPosAngle = 15.0;
        public static final double retractedfeedPosAngle = 120.0;

        private IntakerPivotParams() {}
    }
}
