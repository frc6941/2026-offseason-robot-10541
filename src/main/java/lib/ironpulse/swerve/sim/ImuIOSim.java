package lib.ironpulse.swerve.sim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import lib.ironpulse.swerve.ImuIO;

public class ImuIOSim implements ImuIO {
    public ImuIOSim() {}

    private double lastTimestamp = Timer.getTimestamp();
    private Rotation2d simulatedYawPosition = new Rotation2d();

    @Override
    public void updateInputs(ImuIOInputs inputs) {
        double now = Timer.getTimestamp();
        double dt = now - lastTimestamp;
        lastTimestamp = now;

        inputs.connected = true;
        // Update simulated yaw position based on commanded velocity
        simulatedYawPosition =
                simulatedYawPosition.plus(new Rotation2d(inputs.yawVelocityRadPerSecCmd * dt));
        inputs.yawPosition = simulatedYawPosition;
        inputs.yawVelocityRadPerSec = inputs.yawVelocityRadPerSecCmd;
        inputs.pitchPosition = new Rotation2d();
        inputs.pitchVelocityRadPerSec = 0.0;
        inputs.rollPosition = new Rotation2d();
        inputs.rollVelocityRadPerSec = 0.0;
        inputs.odometryYawTimestamps = new double[] {Timer.getTimestamp()};
        inputs.odometryYawPositions = new Rotation2d[] {inputs.yawPosition};
        inputs.odometryRotations =
                new Rotation3d[] {
                    new Rotation3d(
                            inputs.rollPosition.getMeasure(),
                            inputs.pitchPosition.getMeasure(),
                            inputs.yawPosition.getMeasure())
                };
    }

    @Override
    public void setYawDeg(double yaw) {
        simulatedYawPosition = Rotation2d.fromDegrees(yaw);
    }

    @Override
    public void reset() {
        simulatedYawPosition = new Rotation2d();
        lastTimestamp = Timer.getTimestamp();
    }

    @Override
    public double getYawDeg() {
        return simulatedYawPosition.getDegrees();
    }
}
