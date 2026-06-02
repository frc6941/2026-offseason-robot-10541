package lib.ironpulse.math.filter;

import lombok.Getter;

/**
 * Oscilloscope-style edge detector with confirmation. Triggers only after the signal stays past the
 * threshold for a required number of consecutive samples.
 */
public class EdgeFilter implements DigitalFilter<Double> {
    private double level;
    private double hysteresis;
    private int confirmCount;
    private EdgeType type;
    private double currentValue;
    @Getter private boolean detected;
    private boolean armed;
    private int crossCount;

    /**
     * @param type rising or falling edge
     * @param level absolute trigger level
     * @param hysteresis signal must return past (level +/- hysteresis) before re-arming
     * @param confirmCount consecutive samples past level required to trigger
     */
    public EdgeFilter(EdgeType type, double level, double hysteresis, int confirmCount) {
        this.type = type;
        this.level = level;
        this.hysteresis = hysteresis;
        this.confirmCount = Math.max(1, confirmCount);
        reset(type == EdgeType.RISING ? Double.NEGATIVE_INFINITY : Double.POSITIVE_INFINITY);
    }

    @Override
    public void input(Double value, double dt_s) {
        this.currentValue = value;
    }

    @Override
    public Double compute() {
        detected = false;

        if (type == EdgeType.RISING) {
            // re-arm when signal drops below (level - hysteresis)
            if (currentValue <= level - hysteresis) {
                armed = true;
                crossCount = 0;
            }

            if (armed && currentValue >= level) {
                crossCount++;
                if (crossCount >= confirmCount) {
                    detected = true;
                    armed = false;
                    crossCount = 0;
                }
            } else if (currentValue < level) {
                crossCount = 0;
            }
        } else {
            // re-arm when signal rises above (level + hysteresis)
            if (currentValue >= level + hysteresis) {
                armed = true;
                crossCount = 0;
            }

            if (armed && currentValue <= level) {
                crossCount++;
                if (crossCount >= confirmCount) {
                    detected = true;
                    armed = false;
                    crossCount = 0;
                }
            } else if (currentValue > level) {
                crossCount = 0;
            }
        }

        return detected ? 1.0 : 0.0;
    }

    @Override
    public void reset(Double initVal) {
        currentValue = initVal;
        detected = false;
        crossCount = 0;
        if (type == EdgeType.RISING) {
            armed = initVal < level;
        } else {
            armed = initVal > level;
        }
    }

    public enum EdgeType {
        RISING,
        FALLING
    }
}
