package lib.ironpulse.io;

import java.util.concurrent.atomic.AtomicBoolean;

/** Simple sim beam break IO with a manually settable state. */
public class BeamBreakIOSim implements BeamBreakIO {
    private final AtomicBoolean state = new AtomicBoolean(false);

    public void set(boolean raw) {
        state.set(raw);
    }

    @Override
    public void updateInputs(BeamBreakIOInputs inputs) {
        inputs.rawState = state.get();
        inputs.value = 0.0;
    }
}
