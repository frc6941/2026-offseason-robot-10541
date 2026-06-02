package lib.ironpulse.swerve.mk5n;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import lib.ironpulse.swerve.SwerveConfig;
import lombok.experimental.SuperBuilder;

@SuperBuilder
public class SwerveMK5NConfig extends SwerveConfig {
    public Current driveStatorCurrentLimit;
    public Current steerStatorCurrentLimit;
    public Current driveSupplyCurrentLimit;
    public Current steerSupplyCurrentLimit;
    public Frequency odometryFrequency;
    public CANBus canivoreCanBus;
    public int pigeonId;
}
