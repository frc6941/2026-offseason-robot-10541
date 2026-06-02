package lib.ironpulse.io;

import org.littletonrobotics.junction.AutoLog;

public interface CANCoderIO {
    default void readInputs(CANCoderIOInputs inputs) {}

    @AutoLog
    class CANCoderIOInputs {
        public double positionRotations = 0.0;
        public boolean isConnected = false;
    }
}
