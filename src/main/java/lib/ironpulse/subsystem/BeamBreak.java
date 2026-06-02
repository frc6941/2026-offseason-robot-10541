package lib.ironpulse.subsystem;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.ironpulse.io.BeamBreakIO;
import lib.ironpulse.io.BeamBreakIOInputsAutoLogged;
import lib.ironpulse.utils.TimeDelayedBoolean;

/**
 * Convenience wrapper around {@link BeamBreakIO} that:
 *
 * <p>- Owns the debouncer
 *
 * <p>- Computes {@code inputs.debounced}
 *
 * <p>- Provides wait-style commands (raw/debounced) like the 254/1678 patterns
 */
public class BeamBreak {
    private final BeamBreakIO io;
    private final BeamBreakIOInputsAutoLogged inputs;
    private final Debouncer debouncer;

    public BeamBreak(BeamBreakIO io, Time debounce) {
        this.io = io;
        this.inputs = new BeamBreakIOInputsAutoLogged();
        this.debouncer = new Debouncer(debounce.in(Units.Seconds), DebounceType.kBoth);
    }

    /** Call periodically to refresh inputs and compute debounced value. */
    public void update() {
        io.updateInputs(inputs);
        inputs.debounced = debouncer.calculate(inputs.rawState);
    }

    /** AutoLogged inputs (raw + debounced). */
    public BeamBreakIOInputsAutoLogged getInputs() {
        return inputs;
    }

    /** Raw beam break state. */
    public boolean get() {
        return inputs.rawState;
    }

    /** Inverted raw state (convenience for opposite polarity semantics). */
    public boolean getInverted() {
        return !inputs.rawState;
    }

    /** Debounced beam break state. */
    public boolean getDebounced() {
        return inputs.debounced;
    }

    public Command waitFor(boolean state) {
        return Commands.waitUntil(() -> get() == state);
    }

    public Command waitForDebounced(boolean state) {
        return Commands.waitUntil(() -> getDebounced() == state);
    }

    /**
     * Wait until the beam break stays at {@code state} continuously for {@code seconds}.
     *
     * <p>Note: this relies on {@link #update()} being called periodically while the command runs.
     */
    public Command waitForTime(boolean state, double seconds) {
        TimeDelayedBoolean delay = new TimeDelayedBoolean(seconds);
        return Commands.sequence(
                Commands.runOnce(delay::reset),
                Commands.waitUntil(() -> delay.update(get() == state)));
    }

    /**
     * Wait until the debounced beam break stays at {@code state} continuously for {@code seconds}.
     *
     * <p>Note: this relies on {@link #update()} being called periodically while the command runs.
     */
    public Command waitForDebouncedTime(boolean state, double seconds) {
        TimeDelayedBoolean delay = new TimeDelayedBoolean(seconds);
        return Commands.sequence(
                Commands.runOnce(delay::reset),
                Commands.waitUntil(() -> delay.update(getDebounced() == state)));
    }
}
