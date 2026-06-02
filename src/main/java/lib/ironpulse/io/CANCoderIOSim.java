package lib.ironpulse.io;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public class CANCoderIOSim implements CANCoderIO {

    private Angle currentPosition = Degrees.of(0.0);

    public CANCoderIOSim() {}

    @Override
    public void readInputs(CANCoderIOInputs inputs) {
        inputs.positionRotations = currentPosition.in(Rotations);
        inputs.isConnected = true;
    }

    public void setCurrentPosition(Angle position) {
        currentPosition = position;
    }
}
