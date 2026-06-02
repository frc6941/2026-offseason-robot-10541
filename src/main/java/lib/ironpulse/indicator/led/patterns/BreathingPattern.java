package lib.ironpulse.indicator.led.patterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import lib.ironpulse.indicator.led.AddressableLEDPattern;

/**
 * Represents a breathing LED pattern: the color fades in and out over a full cycle of the given
 * period (in seconds).
 */
public class BreathingPattern implements AddressableLEDPattern {
    private final Color baseColor;
    private final double period;
    private final double startTime;

    /**
     * @param color the color to breathe
     * @param period length of one full fade-in/fade-out cycle, in seconds
     */
    public BreathingPattern(Color color, double period) {
        this.baseColor = color;
        this.period = period;
        this.startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        double elapsed = Timer.getFPGATimestamp() - startTime;
        double phase = 2.0 * Math.PI * (elapsed % period) / period;
        // calculate brightness from 0.0 to 1.0 using a sine wave
        double brightness = (Math.sin(phase - Math.PI / 2.0) + 1.0) * 0.5;

        // apply scaled color to every LED
        Color c =
                new Color(
                        baseColor.red * brightness,
                        baseColor.green * brightness,
                        baseColor.blue * brightness);
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, c);
        }
    }

    @Override
    public boolean isAnimated() {
        return true;
    }
}
