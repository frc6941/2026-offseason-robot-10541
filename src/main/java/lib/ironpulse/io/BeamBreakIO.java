package lib.ironpulse.io;

import org.littletonrobotics.junction.AutoLog;

/** Beam break sensor IO following the AdvantageKit IO pattern. */
public interface BeamBreakIO {
    /** Populate sensor readings. */
    default void updateInputs(BeamBreakIOInputs inputs) {}

    @AutoLog
    class BeamBreakIOInputs {
        /** Raw beam break state as provided by the sensor/implementation. */
        public boolean rawState = false;

        /** Debounced version of {@link #raw}. Typically computed in a wrapper/subsystem. */
        public boolean debounced = false;

        /** Numeric reading from the analog input. */
        public double value = 0.0;
    }
}
