package lib.ironpulse.swerve;

import static edu.wpi.first.units.Units.*;

import frc.robot.SwerveLimiterParamsNT;
import lombok.Getter;

/**
 * Limits the complete composed current against a supply-current budget.
 *
 * <p>Wheel forces become stator amperes through r/(G*Kt). Measured rotor speed and motor winding
 * resistance predict voltage saturation and duty, so stator demand is rated in pack supply amperes
 * before comparison. A common scale preserves the commanded force ratios. Measured non-drive loads
 * are subtracted first when whole-pack current is available; without that measurement the budget is
 * a ceiling on drivetrain draw. Hardware current limits remain active. Invalid pack voltage
 * suppresses all drive current.
 */
public class SwerveLimiter {
    // ---- amp headroom ----

    /**
     * Total pack current the drivetrain plans against, amperes. A direct knob rather than a number
     * derived from a pack model: the model needs an internal resistance that is per-pack and ages,
     * and a budget quietly computed from a stale R is harder to reason about than one somebody set
     * on purpose. The EnergyLab fits put a fresh pack near 450 A to a 7.5 V floor, a mid-life pack
     * near 350, and a tired one near 260 — the right value is a property of the pack in the robot,
     * and this default is the conservative end of that range now that the arm is back on and
     * bidding for the same pack.
     */
    public static final double kDefaultIBudgetAmps = 400.0;

    /** Minimum bus voltage used as a duty denominator, so a brownout cannot divide by ~0. */
    public static final double kDutyMinBusVolts = 6.0;

    /** Share of the bus a module can actually command; the rest is controller headroom. */
    public static final double kDutyMax = 0.95;

    /** Duty is clamped into this band: never zero-rated, never over unity. */
    public static final double kMinDuty = 0.05;

    public static final double kMaxDuty = 1.0;

    /** Current filter time constant — long enough that the budget tracks draw, not ripple. */
    public static final double kCurrentTauS = 0.2;

    private final SwerveConfig config;
    private final double currentPerNewton;

    // filter state
    private double totalAmpsFilt = Double.NaN;
    private double driveAmpsFilt = Double.NaN;

    /** The live budget the drivetrain is planning against. */
    @Getter private double iBudget = Double.NaN;

    /** What is left of that budget for the drivetrain once other loads are paid for. */
    @Getter private double driveHeadroom = Double.NaN;

    /**
     * The drivetrain's deliverable STATOR demand this tick — the whole composed request summed over
     * the modules, after per module voltage saturation is accounted.
     */
    @Getter private double statorDemandAmps = 0.0;

    /** The PACK SUPPLY draw that stator demand implies, each module through its own duty. */
    @Getter private double supplyDemandAmps = 0.0;

    /** Whether the headroom law actually trimmed, as opposed to merely computing what it would. */
    @Getter private boolean headroomApplied = false;

    /** Factor the wrench was scaled by to fit the headroom; 1.0 when it already fitted. */
    @Getter private double wrenchScale = 1.0;

    private final double[] amps;

    public SwerveLimiter(SwerveConfig config) {
        this.config = config;
        this.amps = new double[config.moduleCount()];

        double wheelRadius = config.wheelDiameter.in(Meter) * 0.5;
        this.currentPerNewton = wheelRadius / (config.driveGearRatio * config.driveMotorKt);
    }

    // ------- parameters -------

    /** Whether the current headroom governor is active. */
    public static boolean isEnabled() {
        return SwerveLimiterParamsNT.enabled.getValue();
    }

    public static double iBudgetAmps() {
        return bounded(
                SwerveLimiterParamsNT.iBudgetAmps.getValue(), kDefaultIBudgetAmps, 100.0, 600.0);
    }

    static double bounded(double requested, double fallback, double min, double max) {
        if (!Double.isFinite(requested)) return fallback;
        return edu.wpi.first.math.MathUtil.clamp(requested, min, max);
    }

    // ------- power state -------

    /**
     * Folds this tick's pack measurements into the filtered state the headroom law reads.
     *
     * @param totalAmps total pack current, or NaN when it is not measurable — the budget then
     *     simply cannot subtract other loads, and the drivetrain plans against the whole knob
     * @param driveAmps measured drivetrain supply current, or NaN
     */
    public void updatePower(double totalAmps, double driveAmps) {
        double dt = config.dtS;
        totalAmpsFilt = filterCurrent(totalAmpsFilt, totalAmps, dt);
        driveAmpsFilt = filterCurrent(driveAmpsFilt, driveAmps, dt);

        iBudget = iBudgetAmps();
        // Everything else on the robot is paid first, so what the drivetrain may draw is whatever
        // the budget has left after the other loads. Shedding is a doctrine; this is that doctrine
        // written as arithmetic. With no total measurement there is nothing to subtract, and the
        // budget degrades to a plain ceiling on drive draw rather than to no limit at all.
        double nonDrive =
                Double.isFinite(totalAmpsFilt) && Double.isFinite(driveAmpsFilt)
                        ? Math.max(0.0, totalAmpsFilt - driveAmpsFilt)
                        : 0.0;
        driveHeadroom = iBudget - nonDrive;
    }

    private static double filterCurrent(double state, double sample, double dt) {
        if (!Double.isFinite(sample)) return state;
        if (!Double.isFinite(state)) return sample;
        return state + dt / (kCurrentTauS + dt) * (sample - state);
    }

    // ------- the per tick limit chain -------

    /** The stator current a given wheel force takes, {@code r / (G·Kt)}. One place owns this. */
    public double statorAmpsForForce(double forceNewton) {
        return forceNewton * currentPerNewton;
    }

    /**
     * Bounds a set of wheel FORCES, converting them to stator current first. Same chain as {@link
     * #applyComposedAmps}, entered a step earlier — for callers that hold the model's forces rather
     * than an already-composed current.
     *
     * @param forcesNewton the feedforward's allocated wheel forces, already trimmed by its scale
     * @param rotorRadPerSec measured per module drive ROTOR speed, which sets both back EMF and the
     *     deliverable ceiling, or null to assume a stalled rotor
     * @param busVolts measured pack voltage
     * @return per module drive STATOR current in amperes; the same buffer each tick
     */
    public double[] apply(double[] forcesNewton, double[] rotorRadPerSec, double busVolts) {
        int moduleCount = config.moduleCount();
        for (int i = 0; i < moduleCount; i++) amps[i] = statorAmpsForForce(forcesNewton[i]);
        return runChain(rotorRadPerSec, busVolts);
    }

    /**
     * Bounds an already-composed total current: feedforward, the software P term and friction, all
     * summed on the RIO. This is the drive path. Because the whole commanded current passes through
     * here, the headroom law bounds the drivetrain's <i>real</i> draw by construction, not just the
     * feedforward's share of it.
     *
     * @param statorAmps the composed per module stator current, amperes
     * @return per module limited stator current; the same buffer each tick
     */
    public double[] applyComposedAmps(
            double[] statorAmps, double[] rotorRadPerSec, double busVolts) {
        int moduleCount = config.moduleCount();
        for (int i = 0; i < moduleCount; i++) amps[i] = statorAmps[i];
        return runChain(rotorRadPerSec, busVolts);
    }

    /**
     * The headroom scale on the total, on whatever stator currents are loaded into {@code amps}.
     */
    private double[] runChain(double[] rotorRadPerSec, double busVolts) {
        int moduleCount = config.moduleCount();
        if (!Double.isFinite(busVolts) || busVolts <= 0.0) {
            java.util.Arrays.fill(amps, 0.0);
            supplyDemandAmps = 0.0;
            statorDemandAmps = 0.0;
            wrenchScale = 0.0;
            headroomApplied = true;
            return amps;
        }
        for (int i = 0; i < moduleCount; i++) if (!Double.isFinite(amps[i])) amps[i] = 0.0;

        // Fit the whole wrench inside the drivetrain's share of the pack. Stator amps are converted
        // to supply amps per module first — see the units note on the class — and one factor is
        // applied to all four, which keeps the force ratios the allocation solved for so the
        // chassis still accelerates in the commanded direction, just less hard.
        double denominator = Math.max(busVolts, kDutyMinBusVolts);
        double kE = config.driveMotorKt;
        double motorR = config.driveMotorR;
        supplyDemandAmps = 0.0;
        statorDemandAmps = 0.0;
        for (int i = 0; i < moduleCount; i++) {
            double backEmf = kE * Math.abs(rotorSpeedOf(rotorRadPerSec, i));
            // What the motor could actually push through itself against its own back EMF. A
            // request beyond this is not current the pack will ever be asked for.
            double deliverable = Math.max(0.0, (kDutyMax * busVolts - backEmf) / motorR);
            double demand = Math.min(Math.abs(amps[i]), deliverable);
            double duty =
                    edu.wpi.first.math.MathUtil.clamp(
                            (backEmf + demand * motorR) / denominator, kMinDuty, kMaxDuty);
            statorDemandAmps += demand;
            supplyDemandAmps += demand * duty;
        }

        wrenchScale = 1.0;
        headroomApplied = false;
        if (Double.isFinite(driveHeadroom)
                && supplyDemandAmps > driveHeadroom
                && supplyDemandAmps > 0.0) {
            wrenchScale = Math.max(0.0, driveHeadroom) / supplyDemandAmps;
            // The law computes what it would do whatever the switch says, so a bag can be judged
            // before the governor is allowed to touch the robot. Only the trim itself is gated.
            if (isEnabled()) {
                headroomApplied = true;
                for (int i = 0; i < moduleCount; i++) amps[i] *= wrenchScale;
                supplyDemandAmps *= wrenchScale;
                statorDemandAmps *= wrenchScale;
            }
        }
        return amps;
    }

    /** Measured rotor speed, or a stalled rotor when the measurement is missing. */
    private static double rotorSpeedOf(double[] rotorRadPerSec, int index) {
        if (rotorRadPerSec == null
                || index >= rotorRadPerSec.length
                || !Double.isFinite(rotorRadPerSec[index])) return 0.0;
        return rotorRadPerSec[index];
    }
}
