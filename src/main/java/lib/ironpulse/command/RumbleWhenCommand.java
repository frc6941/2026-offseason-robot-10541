package lib.ironpulse.command;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

public class RumbleWhenCommand extends Command {
    private final GenericHID[] hid;
    private final BooleanSupplier rumbleCondition;

    public RumbleWhenCommand(BooleanSupplier rumbleCondition, GenericHID... hid) {
        this.hid = hid;
        this.rumbleCondition = rumbleCondition;
    }

    @Override
    public void execute() {
        for (var i : hid) {
            i.setRumble(GenericHID.RumbleType.kBothRumble, rumbleCondition.getAsBoolean() ? 1 : 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        for (var i : hid) {
            i.setRumble(GenericHID.RumbleType.kBothRumble, 0);
        }
    }
}
