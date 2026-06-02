package lib.ironpulse.indicator;

import lib.ironpulse.indicator.led.AddressableLEDWrapper;

public class IndicatorIOARGB implements IndicatorIO {
    private final AddressableLEDWrapper led;
    private Patterns currentPattern = Patterns.NORMAL;

    public IndicatorIOARGB(int port, int length) {
        led = new AddressableLEDWrapper(port, length);
        led.setIntensity(1.0);
        led.start(0.02);
    }

    @Override
    public void updateInputs(IndicatorIOInputs inputs) {
        setPattern(currentPattern);
        inputs.currentPattern = currentPattern;
    }

    @Override
    public void setPattern(Patterns pattern) {
        currentPattern = pattern;
        led.setPattern(pattern.pattern);
    }

    @Override
    public void reset() {
        led.stop();
        led.start(0.02);
    }
}
