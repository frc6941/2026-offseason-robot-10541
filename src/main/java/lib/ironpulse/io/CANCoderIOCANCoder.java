package lib.ironpulse.io;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import lib.ironpulse.utils.PhoenixUtils;

public class CANCoderIOCANCoder implements CANCoderIO {
    private final CANcoder encoder;
    private StatusSignal<Angle> positionSig;

    public CANCoderIOCANCoder(int canDeviceId, CANBus mBus, Angle offset, boolean inverted) {
        encoder = new CANcoder(canDeviceId, mBus);

        var cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = offset.in(Rotations);
        cancoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        cancoderConfigs.MagnetSensor.SensorDirection =
                inverted
                        ? SensorDirectionValue.Clockwise_Positive
                        : SensorDirectionValue.CounterClockwise_Positive;
        PhoenixUtils.tryUntilOk(5, () -> encoder.getConfigurator().apply(cancoderConfigs));
        positionSig = encoder.getAbsolutePosition();
        positionSig.setUpdateFrequency(1000.0);
        PhoenixUtils.registerSignals(mBus, new BaseStatusSignal[] {positionSig});
        encoder.optimizeBusUtilization();
    }

    @Override
    public void readInputs(CANCoderIOInputs inputs) {

        // Update the inputs
        inputs.positionRotations = positionSig.getValueAsDouble();
        inputs.isConnected = BaseStatusSignal.isAllGood(positionSig);
    }
}
