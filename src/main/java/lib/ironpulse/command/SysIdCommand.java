package lib.ironpulse.command;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lib.ironpulse.subsystem.MotorSubsystem;
import lib.ntext.NTParameter;
import org.littletonrobotics.junction.Logger;

public class SysIdCommand {
    private static final String kLogPrefix = "SysId";
    private final SysIdRoutine routine;
    private final MotorSubsystem<?, ?> subsystem;

    public SysIdCommand(MotorSubsystem<?, ?> subsystem) {
        this.subsystem = subsystem;
        this.routine = buildRoutine(subsystem);
    }

    public Command quasistatic(SysIdRoutine.Direction direction) {
        return requireSubsystem(routine.quasistatic(direction));
    }

    public Command dynamic(SysIdRoutine.Direction direction) {
        return requireSubsystem(routine.dynamic(direction));
    }

    private SysIdRoutine buildRoutine(MotorSubsystem<?, ?> subsystem) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Seconds).of(SysIdCommandParams.rampRateVoltsPerSecond),
                        Volts.of(SysIdCommandParams.stepVoltageVolts),
                        Seconds.of(SysIdCommandParams.timeoutSeconds),
                        state ->
                                Logger.recordOutput(
                                        kLogPrefix + "/" + subsystem.getName() + "/State",
                                        state.toString())),
                new SysIdRoutine.Mechanism(subsystem::setVoltage, null, subsystem));
    }

    private Command requireSubsystem(Command command) {
        return Commands.sequence(Commands.runOnce(() -> {}, subsystem), command);
    }

    @NTParameter(tableName = "Params/Commands/SysId")
    public static final class SysIdCommandParams {
        public static final double rampRateVoltsPerSecond = 1;
        public static final double stepVoltageVolts = 7.0;
        public static final double timeoutSeconds = 20.0;

        private SysIdCommandParams() {}
    }
}
