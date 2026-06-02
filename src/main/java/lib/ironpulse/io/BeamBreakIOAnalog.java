package lib.ironpulse.io;

import edu.wpi.first.wpilibj.AnalogInput;
import java.util.function.BooleanSupplier;

/** "Real" beam break IO backed by a {@link BooleanSupplier} (DigitalInput, CAN sensor, etc.). */
public class BeamBreakIOAnalog implements BeamBreakIO {

    private final double voltageThreshold;
    private final AnalogInput input;

    public BeamBreakIOAnalog(int channel) {
        this(channel, 3.0);
    }

    public BeamBreakIOAnalog(int channel, double voltageThreshold) {
        this.input = new AnalogInput(channel);
        this.voltageThreshold = voltageThreshold;
    }

    @Override
    public void updateInputs(BeamBreakIOInputs inputs) {
        inputs.rawState = input.getVoltage() > voltageThreshold;
        inputs.value = input.getVoltage();
    }
}
