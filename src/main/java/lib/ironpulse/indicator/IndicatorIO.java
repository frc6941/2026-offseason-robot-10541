package lib.ironpulse.indicator;

import edu.wpi.first.wpilibj.util.Color;
import lib.ironpulse.indicator.led.AddressableLEDPattern;
import lib.ironpulse.indicator.led.patterns.*;
import org.littletonrobotics.junction.AutoLog;

public interface IndicatorIO {
    default void updateInputs(IndicatorIOInputs inputs) {}

    default void setPattern(Patterns pattern) {}

    default void reset() {}

    enum Patterns {
        LOSS(new BlinkingPattern(Color.kOrange, 1.0)),
        RED_ALLIANCE(new ScannerPattern(Color.kRed, 8)),
        BLUE_ALLIANCE(new ScannerPattern(Color.kBlue, 8)),
        NORMAL(new ScannerPattern(Color.kBlue, 8)),

        INTAKE(new BlinkingPattern(Color.kBlue, 0.04)),

        HOLD_SHOOTING(new BlinkingPattern(Color.kPurple, 0.04)),
        RESET_ODOM(new BlinkingPattern(Color.kWhite, 0.1)),
        SHOOTING(new BlinkingPattern(Color.kRed, 0.04)),
        AFTER_SHOOTING(new BlinkingPattern(Color.kGreen, 0.04)),
        AUTO(new RainbowingPattern()),

        CLIMB_DEPLOYED(new BlinkingPattern(Color.kWhite, 0.2)),
        CLIMB_FINISHED(new RainbowingPattern()),

        OPPONENT_SHIFT_WARNING(new BlinkingPattern(Color.kYellow, 0.2));

        public final AddressableLEDPattern pattern;

        Patterns(AddressableLEDPattern color) {
            this.pattern = color;
        }
    }

    @AutoLog
    class IndicatorIOInputs {
        public Patterns currentPattern = Patterns.NORMAL;
    }
}
