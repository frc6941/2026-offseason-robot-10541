package frc.robot.subsystems.Configs;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotConstants.CANIVORE_CAN_BUS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.RobotConstants;
import lib.ironpulse.swerve.ImuPigeonConfig;
import lib.ironpulse.swerve.SwerveConfig;
import lib.ironpulse.swerve.SwerveLimit;
import lib.ironpulse.swerve.SwerveModuleLimit;
import lib.ironpulse.swerve.mk5n.SwerveMK5NConfig;
import lib.ironpulse.swerve.sim.SwerveSimConfig;
import lib.ntext.NTParameter;

public final class SwerveMK5Config {

    // Half-dimensions from robot center to module (meters)
    public static final double kSwerveHalfLength = 0.3429;
    public static final double kSwerveHalfWidth  = 0.3429;

    // TODO: tune Pigeon IMU mounting angles for this robot (currently copied from competition robot)
    public static final ImuPigeonConfig pigeonConfig =
            ImuPigeonConfig.builder()
                    .mountPoseYaw(-24.29374389648438)
                    .mountPosePitch(-0.0090445756912231)
                    .mountPoseRoll(-4.7371618461608887)
                    .gyroScalarZ(-3.5)
                    .build();

    // Per-module physical limits (MK5n R1 + Kraken X60/X44 with FOC)
    public static final SwerveModuleLimit kDefaultSwerveModuleLimit =
            SwerveModuleLimit.builder()
                    .maxDriveVelocity(InchesPerSecond.of(5800 / 60.0 / 7.03 * Math.PI * 4.0))
                    .maxDriveAcceleration(MetersPerSecondPerSecond.of(200))
                    .maxSteerAngularVelocity(RotationsPerSecond.of(7368.0 / 60.0 / (287.0 / 11.0)))
                    .maxSteerAngularAcceleration(
                            RotationsPerSecondPerSecond.of(7008.0 / 60.0 / (287.0 / 11.0) / 0.2))
                    .build();

    // Chassis-level limits
    public static final SwerveLimit kDefaultSwerveLimit =
            SwerveLimit.builder()
                    .maxLinearVelocity(MetersPerSecond.of(4.0))
                    .maxSkidAcceleration(MetersPerSecondPerSecond.of(200))
                    .maxAngularVelocity(DegreesPerSecond.of(450))
                    .maxAngularAcceleration(DegreesPerSecondPerSecond.of(5000))
                    .build();

    // Module configs — encoder offsets are from the competition robot and will need re-tuning
    public static final SwerveConfig.SwerveModuleConfig kModuleFL =
            SwerveConfig.SwerveModuleConfig.builder()
                    .name("LF")
                    .location(new Translation2d(kSwerveHalfLength, kSwerveHalfWidth))
                    .driveMotorId(9).steerMotorId(8).encoderId(7)
                    .driveMotorEncoderOffset(Degree.of(0))
                    .steerMotorEncoderOffset(Rotations.of(0.412354))
                    .driveInverted(false).steerInverted(false).encoderInverted(false)
                    .build();

    public static final SwerveConfig.SwerveModuleConfig kModuleFR =
            SwerveConfig.SwerveModuleConfig.builder()
                    .name("RF")
                    .location(new Translation2d(kSwerveHalfLength, -kSwerveHalfWidth))
                    .driveMotorId(12).steerMotorId(11).encoderId(10)
                    .driveMotorEncoderOffset(Degree.of(0))
                    .steerMotorEncoderOffset(Rotations.of(0.373291))
                    .driveInverted(true).steerInverted(false).encoderInverted(false)
                    .build();

    public static final SwerveConfig.SwerveModuleConfig kModuleBL =
            SwerveConfig.SwerveModuleConfig.builder()
                    .name("LB")
                    .location(new Translation2d(-kSwerveHalfLength, kSwerveHalfWidth))
                    .driveMotorId(6).steerMotorId(5).encoderId(4)
                    .driveMotorEncoderOffset(Degree.of(0))
                    .steerMotorEncoderOffset(Rotations.of(0.076172))
                    .driveInverted(false).steerInverted(false).encoderInverted(false)
                    .build();

    public static final SwerveConfig.SwerveModuleConfig kModuleBR =
            SwerveConfig.SwerveModuleConfig.builder()
                    .name("RB")
                    .location(new Translation2d(-kSwerveHalfLength, -kSwerveHalfWidth))
                    .driveMotorId(3).steerMotorId(2).encoderId(1)
                    .driveMotorEncoderOffset(Degree.of(0))
                    .steerMotorEncoderOffset(Rotations.of(-0.180420))
                    .driveInverted(true).steerInverted(false).encoderInverted(false)
                    .build();

    // Simulation config
    public static final SwerveSimConfig kSimConfig =
            SwerveSimConfig.builder()
                    .name("Swerve")
                    .dtS(RobotConstants.LOOPER_DT)
                    .wheelDiameter(Inch.of(4.0))
                    .driveGearRatio(6.03)
                    .steerGearRatio(287.0 / 11.0)
                    .driveMotorKt(0.0182)
                    .driveMass(Kilograms.of(22))
                    .driveMotor(DCMotor.getKrakenX60Foc(1))
                    .driveMomentOfInertia(KilogramSquareMeters.of(0.04))
                    .driveStdDevPos(0.0000001)
                    .driveStdDevVel(0.000001)
                    .steerMotor(DCMotor.getKrakenX44Foc(1))
                    .steerMomentOfInertia(KilogramSquareMeters.of(0.01))
                    .steerStdDevPos(0.0000001)
                    .steerStdDevVel(0.000001)
                    .defaultSwerveLimit(kDefaultSwerveLimit)
                    .defaultSwerveModuleLimit(kDefaultSwerveModuleLimit)
                    .moduleConfigs(new SwerveConfig.SwerveModuleConfig[]{
                            kModuleFL, kModuleFR, kModuleBL, kModuleBR
                    })
                    .build();

    // Full real-robot config
    public static final SwerveMK5NConfig kRealConfig =
            SwerveMK5NConfig.builder()
                    .name("Swerve")
                    .dtS(RobotConstants.LOOPER_DT)
                    .wheelDiameter(Inch.of(4.0))
                    .driveGearRatio(6.03)
                    .steerGearRatio(287.0 / 11.0)
                    .driveMotorKt(0.0182)
                    .driveMass(Kilograms.of(22))
                    .pigeonConfig(pigeonConfig)
                    .defaultSwerveLimit(kDefaultSwerveLimit)
                    .defaultSwerveModuleLimit(kDefaultSwerveModuleLimit)
                    .moduleConfigs(new SwerveConfig.SwerveModuleConfig[]{
                            kModuleFL, kModuleFR, kModuleBL, kModuleBR
                    })
                    .odometryFrequency(Hertz.of(100))
                    .driveStatorCurrentLimit(Amps.of(100))
                    .driveSupplyCurrentLimit(Amps.of(65))
                    .steerStatorCurrentLimit(Amps.of(45))
                    .steerSupplyCurrentLimit(Amps.of(35))
                    .canivoreCanBus(CANIVORE_CAN_BUS)
                    .pigeonId(RobotConstants.PIGEON_ID)
                    .build();

    // Swerve module PID/FF gains moved to frc.robot.SwerveModuleParams (package frc.robot) — the
    // vendored lib imports frc.robot.SwerveModuleParamsNT directly.
}
