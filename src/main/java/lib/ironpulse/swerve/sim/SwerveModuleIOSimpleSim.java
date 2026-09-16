package lib.ironpulse.swerve.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import lib.ironpulse.swerve.SwerveModuleIO;
import lib.ironpulse.utils.Logging;

public class SwerveModuleIOSimpleSim implements SwerveModuleIO {
    /**
     * Viscous drag time constant for the current-driven wheel, seconds.
     *
     * <p>Integrating commanded current with nothing opposing it gives a wheel that accelerates
     * forever, so the loop would never settle and the sim would report a steady-state error the
     * real robot does not have. This stands in for rolling resistance and driveline drag — the load
     * the composition's {@code kS}/{@code kV} terms exist to pay for — and is sized so it costs
     * about what they supply: at 4.5 m/s the friction feedforward is ≈23 A, which through {@link
     * #accelPerAmp} is ≈3.5 m/s², and {@code v / a} ≈ 1.25 s. It is a plausibility number for a
     * kinematic stand-in, not a measured drivetrain parameter.
     */
    private static final double kDragTauS = 1.25;

    private final SwerveSimConfig config;
    private final SwerveModuleState simState = new SwerveModuleState();
    private final SwerveModulePosition simPosition = new SwerveModulePosition();
    private double prevTimestamp;

    /**
     * Wheel-frame acceleration per commanded stator amp, {@code Kt·G/r} over the module's share.
     */
    private final double accelPerAmp;

    /** Last commanded torque current, and whether the drive is being driven by current at all. */
    private double driveCommandAmps = 0.0;

    private boolean driveCurrentMode = false;

    public SwerveModuleIOSimpleSim(SwerveSimConfig config, int idx) {
        this.config = config;
        double wheelRadius = config.wheelDiameter.in(Meter) * 0.5;
        double massPerModule = config.driveMass.in(Kilograms) / config.moduleCount();
        this.accelPerAmp =
                config.driveMotorKt * config.driveGearRatio / wheelRadius / massPerModule;
        // record initial time for simulation
        prevTimestamp = Timer.getTimestamp();
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs data) {
        double now = Timer.getTimestamp();
        double dt = now - prevTimestamp;
        prevTimestamp = now;

        // The drive motor takes raw torque current, so unlike the velocity commands below there is
        // no speed to copy into the state — it has to be integrated, or a current-commanded sim
        // simply never moves. First order: commanded amps become wheel acceleration through
        // Kt·G/r against the module's share of the chassis mass, minus the drag term.
        if (driveCurrentMode && dt > 0.0) {
            double accel =
                    driveCommandAmps * accelPerAmp - simState.speedMetersPerSecond / kDragTauS;
            simState.speedMetersPerSecond += accel * dt;
        }

        // integrate drive distance
        simPosition.distanceMeters += simState.speedMetersPerSecond * dt;
        // update module heading
        simPosition.angle = simState.angle;

        double driveVel = simState.speedMetersPerSecond;
        double steerVel =
                dt > 0 ? (simPosition.angle.getRadians() - data.steerMotorPositionRad) / dt : 0.0;

        data.driveMotorConnected = true;
        data.driveMotorPositionRad =
                simPosition.distanceMeters * 2.0 / config.wheelDiameter.in(Meter);
        data.driveMotorPositionRadSamples = new double[] {data.driveMotorPositionRad};
        // Convert wheel linear velocity (m/s) to wheel angular velocity (rad/s).
        // angle(rad) = distance / radius  =>  omega(rad/s) = v(m/s) / r(m) = v * 2 / diameter
        data.driveMotorVelocityRadPerSec = driveVel * 2.0 / config.wheelDiameter.in(Meter);
        data.driveMotorTorqueCurrentAmpere = driveCurrentMode ? driveCommandAmps : 0.0;

        data.steerMotorConnected = true;
        data.steerMotorPositionRad = simPosition.angle.getRadians();
        data.steerMotorPositionRadSamples = new double[] {data.steerMotorPositionRad};
        data.steerMotorVelocityRadPerSec = steerVel;
    }

    @Override
    public void setDriveOpenLoop(Voltage des) {
        driveCurrentMode = false;
        simState.speedMetersPerSecond = des.in(Volts) / RobotController.getBatteryVoltage() * 4.0;
    }

    @Override
    public void setDriveVelocity(LinearVelocity linearVelocityDes) {
        driveCurrentMode = false;
        simState.speedMetersPerSecond = linearVelocityDes.in(MetersPerSecond);
    }

    @Override
    public void setDriveCurrent(Current amps) {
        driveCurrentMode = true;
        driveCommandAmps = amps.in(Amps);
    }

    @Override
    public void setSteerAngleAbsolute(Angle des) {
        simState.angle = new Rotation2d(des);
        simPosition.angle = simState.angle;
    }

    @Override
    public void configDriveController(
            double kp, double ki, double kd, double ks, double kv, double ka) {
        Logging.info(
                config.name + "Module",
                "Module drive controller updated! kp: %.2f, ki: %.2f, kd: %.2f",
                kp,
                ki,
                kd);
    }

    @Override
    public void configSteerController(double kp, double ki, double kd, double ks) {
        Logging.info(
                config.name + "Module",
                "Module steer controller updated! kp: %.2f, ki: %.2f, kd: %.2f",
                kp,
                ki,
                kd);
    }
}
