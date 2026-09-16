package lib.ironpulse.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.Arrays;

/**
 * Per-wheel traction control against an independently estimated chassis velocity.
 *
 * <p>Excess wheel speed and slip ratio engage the governor. A slipping wheel's P feedback uses the
 * body reference, and its complete current is scaled. Torque sign prevents holding a wheel below
 * the chassis while it is being driven. Release and common-mode acceleration detection use physical
 * time; steering coupling temporarily excludes uncertain wheel measurements.
 */
public final class TractionGovernor {
    /** The knobs, read by the caller from {@code Params/Swerve/Traction}. */
    public record Limits(
            boolean enabled,
            double lambdaStar,
            double dvHi,
            double kP,
            double scaleMin,
            double releaseS,
            double vFloor,
            double cmAccel,
            double leadS) {}

    /**
     * The chassis the wheels are judged against: body velocity (m/s), its acceleration (m/s²), and
     * the rate the frame turns at (rad/s), all robot frame — and whether there is one at all.
     * {@code valid} false means {@link BodyVelocityEstimator} had no IMU behind it this tick.
     */
    public record Reference(
            double vx, double vy, double ax, double ay, double omega, boolean valid) {}

    /**
     * Slip ratio the P law holds an engaged wheel at: the carpet's μ-peak.
     *
     * <p>0.35 after the first enabled drive. Quiet |λ| runs to 0.20–0.26 at p90 while real slip
     * sits at 0.8 p50, so anything nearer the noise spends the law's authority on wheels that are
     * not slipping — at 0.20/0.15 a third of engaged wheel-ticks came out pinned at the floor, and
     * the enabled run still put 36 % of wheel-ticks under a cut with 17 % of them at the floor.
     */
    public static final double kDefaultLambdaStar = 0.35;

    /** Excess over the reference, m/s, that engages outright — the 08-26 body-reference sweep. */
    public static final double kDefaultDvHi = 0.5;

    /** Scale lost per unit of |λ| over lambdaStar; 1.0 after the enabled run, which was sharp. */
    public static final double kDefaultKP = 1.0;

    /**
     * The scale never falls under this: an engaged wheel keeps a share, not nothing. 0.25 after
     * driving it live from 2026-08-27, where a quarter of the current turned out to be enough to
     * arrest a wheel and 0.50 was leaving it spinning: on the acrylic test the catch moved to the
     * first tick and peak draw fell from 72 A to 28 A, with no over-cut and the launch delivered
     * unchanged. The floor is where the cuts pile up, so it is the number the drive actually feels.
     */
    public static final double kDefaultScaleMin = 0.25;

    /**
     * Seconds under dvHi/2 before release, and the ramp back to 1. 0.10 from the same sessions: a
     * wheel that has just been caught is still the wheel most likely to break away again, so the
     * hold is worth more than the promptness of letting go.
     */
    public static final double kDefaultReleaseS = 0.100;

    public static final double kDefaultVFloor = 0.3;

    /** Chassis acceleration, m/s², over the common-mode window that engages every wheel. */
    public static final double kDefaultCmAccel = 20.0;

    /** Seconds the reference is predicted ahead, paying the IMU's lag against the wheels. */
    public static final double kDefaultLeadS = 0.020;

    public static final int kTriggerNone = 0;
    public static final int kTriggerExcess = 1;
    public static final int kTriggerReverse = 2;
    public static final int kTriggerCommon = 3;

    /**
     * Heading error, radians, past which a wheel's speed is not measuring what it is compared to.
     * {@link BodyVelocityEstimator} gates its own measurement update on the same test — a wheel
     * that cannot be judged against the reference cannot be allowed to move it either.
     */
    static final double kJudgeHeadingErr = Math.toRadians(10.0);

    /**
     * Ticks a wheel stays unjudged for after steer coupling or a reference that was not there: this
     * one and the two before it.
     */
    static final int kBlindTicks = 3;

    /**
     * The window the common-mode trigger measures chassis acceleration over, seconds. Long enough
     * to clear the wheels' one-tick noise (p90 11 m/s² at one tick, 5.6 over 25 ms) and short
     * enough to catch a launch inside it. The secant is taken on the robot-frame twist, so it
     * carries the frame's own rotation; at the profile's caps that term stays well under {@code
     * cmAccel}.
     */
    static final double kCommonModeS = 0.025;

    private final int moduleCount;
    private final double dt;
    private final double[] rx;
    private final double[] ry;

    /** The last {@link #kCommonModeS} of the wheels' twist, for the common-mode secant. */
    private final double[] histFkX;

    private final double[] histFkY;
    private long ticks;

    private final double[] lambda;
    private final double[] excess;
    private final double[] vRef;
    private final double[] scale;
    private final double[] measuredForP;
    private final boolean[] engaged;
    private final boolean[] judged;
    private final int[] trigger;
    private boolean inputsFinite = true;
    private boolean cmTrigger = false;

    private final double[] law;

    /**
     * Ticks each engaged wheel has been quiet for. Counted, not accumulated in seconds: summing dt
     * lands the release a tick late whenever releaseS/dt does not come out exact in binary.
     */
    private final int[] calmTicks;

    private int steerCoupledTicks;
    private int refInvalidTicks;

    public TractionGovernor(Translation2d[] moduleLocations, double dtS) {
        moduleCount = moduleLocations.length;
        dt = dtS;
        rx = new double[moduleCount];
        ry = new double[moduleCount];
        for (int i = 0; i < moduleCount; i++) {
            rx[i] = moduleLocations[i].getX();
            ry[i] = moduleLocations[i].getY();
        }
        int window = Math.max(1, (int) Math.round(kCommonModeS / dt));
        histFkX = new double[window + 1];
        histFkY = new double[window + 1];

        lambda = new double[moduleCount];
        excess = new double[moduleCount];
        vRef = new double[moduleCount];
        scale = new double[moduleCount];
        measuredForP = new double[moduleCount];
        engaged = new boolean[moduleCount];
        judged = new boolean[moduleCount];
        trigger = new int[moduleCount];
        law = new double[moduleCount];
        calmTicks = new int[moduleCount];
        reset();
    }

    /** Forgets every episode: nothing carries across a disable. */
    public void reset() {
        Arrays.fill(lambda, 0.0);
        Arrays.fill(excess, 0.0);
        Arrays.fill(vRef, 0.0);
        Arrays.fill(scale, 1.0);
        Arrays.fill(measuredForP, 0.0);
        Arrays.fill(engaged, false);
        Arrays.fill(judged, false);
        Arrays.fill(trigger, kTriggerNone);
        Arrays.fill(law, 1.0);
        Arrays.fill(calmTicks, 0);
        inputsFinite = true;
        cmTrigger = false;
        steerCoupledTicks = 0;
        refInvalidTicks = 0;
        ticks = 0;
    }

    /**
     * One tick of the law.
     *
     * @param measuredMps per module measured drive speed, m/s, signed along the heading
     * @param headings per module measured heading
     * @param headingsCmd per module commanded heading — how far from measuring the reference's
     *     direction each wheel is
     * @param ampsCmdPrev per module composed drive current before the governor's own scale, from
     *     the previous tick — this tick's is not composed yet, and the composition depends on what
     *     is decided here. 5 ms, and only its sign is read
     * @param steerCoupled whether modules still turning withheld part of this tick's step
     * @param fk the wheels' own twist, robot frame, for the common-mode secant only
     * @param reference the chassis the wheels are judged against
     * @param limits the live knobs
     */
    public void update(
            double[] measuredMps,
            Rotation2d[] headings,
            Rotation2d[] headingsCmd,
            double[] ampsCmdPrev,
            boolean steerCoupled,
            ChassisSpeeds fk,
            Reference reference,
            Limits limits) {
        steerCoupledTicks = steerCoupled ? kBlindTicks : Math.max(0, steerCoupledTicks - 1);
        refInvalidTicks = reference.valid() ? Math.max(0, refInvalidTicks - 1) : kBlindTicks;
        // The ring is still fed, so the secant is whole again as soon as the reference is: the
        // trigger is off, not blind. Nothing may engage on it while the wheels it would engage are
        // going to be scaled by a λ measured against a reference that was not there.
        cmTrigger = commonMode(fk, limits) && refValid();
        inputsFinite = true;

        double bodyX = reference.vx() + reference.ax() * limits.leadS();
        double bodyY = reference.vy() + reference.ay() * limits.leadS();
        for (int i = 0; i < moduleCount; i++) {
            double cos = headings[i].getCos();
            double sin = headings[i].getSin();
            vRef[i] =
                    (bodyX - reference.omega() * ry[i]) * cos
                            + (bodyY + reference.omega() * rx[i]) * sin;
            excess[i] = measuredMps[i] - vRef[i];
            lambda[i] = excess[i] / Math.max(Math.abs(vRef[i]), limits.vFloor());
            judged[i] =
                    refValid()
                            && steerCoupledTicks == 0
                            && headingError(headings[i], headingsCmd[i]) < kJudgeHeadingErr;

            if (!Double.isFinite(lambda[i])) {
                // A wheel nobody can judge is left alone, and the log says so.
                inputsFinite = false;
                lambda[i] = Double.NaN;
                judged[i] = false;
            }
            hold(i, limits);
            if (engaged[i] && excess[i] * Math.signum(ampsCmdPrev[i]) <= 0.0) {
                // The torque is not driving this error, so there is nothing here to cut.
                releaseAgainstTheTorque(i);
            }

            // The P law while engaged, 1 otherwise; the scale may drop at once but only ever
            // recovers at 1/releaseS, which is the release ramp and the same bound inside an
            // episode as λ falls back.
            double target =
                    engaged[i]
                            ? edu.wpi.first.math.MathUtil.clamp(
                                    1.0
                                            - limits.kP()
                                                    * Math.max(
                                                            Math.abs(lambda[i])
                                                                    - limits.lambdaStar(),
                                                            0.0),
                                    limits.scaleMin(),
                                    1.0)
                            : 1.0;
            law[i] = Math.min(target, law[i] + dt / limits.releaseS());
            scale[i] = limits.enabled() ? law[i] : 1.0;
            measuredForP[i] = limits.enabled() && engaged[i] ? vRef[i] : measuredMps[i];
        }
        ticks++;
    }

    /**
     * Lets wheel {@code i} go, at once and without the release ramp, because the current commanded
     * to it is already pulling it back toward ground speed.
     *
     * <p>A wheel is worth cutting only when its velocity error is on the same side as the torque
     * driving it. That single test is what separates the four cases the excess alone cannot:
     *
     * <ul>
     *   <li>launch over-spin — wheel faster than the ground, drive current positive: same sign,
     *       cut;
     *   <li>a wheel held <i>below</i> ground speed on a launch, drive current still positive:
     *       opposite signs, so cutting it further is the governor fighting its own drivetrain. This
     *       is the case that cost 26 % of engaged launch wheel-ticks on 2026-08-26;
     *   <li>brake lock-up — chassis rolling on, wheel stopped, braking current negative: same sign,
     *       cut, which is what an ABS does;
     *   <li>reverse spin-up at a stop — wheel driven backwards through zero against a chassis still
     *       rolling forwards: same sign, cut.
     * </ul>
     *
     * <p>Deliberately not the simpler {@code excess ≤ 0 → release}: that reads the last two cases
     * as "the wheel is slower than the chassis, leave it alone" and lets a locked or reversing
     * wheel run.
     */
    private void releaseAgainstTheTorque(int i) {
        engaged[i] = false;
        calmTicks[i] = 0;
        trigger[i] = kTriggerNone;
        law[i] = 1.0;
    }

    /** Engages, releases, and states what holds wheel {@code i} this tick. */
    private void hold(int i, Limits limits) {
        int condition = judged[i] ? condition(i, limits) : kTriggerNone;
        if (!judged[i]) {
            engaged[i] = false;
            calmTicks[i] = 0;
        } else if (engaged[i]) {
            if (condition != kTriggerNone) calmTicks[i] = 0;
            else {
                calmTicks[i] = Math.abs(excess[i]) < 0.5 * limits.dvHi() ? calmTicks[i] + 1 : 0;
                if (calmTicks[i] >= Math.max(1, (int) Math.round(limits.releaseS() / dt)))
                    engaged[i] = false;
            }
        } else if (condition != kTriggerNone) {
            engaged[i] = true;
            calmTicks[i] = 0;
        }
        trigger[i] = engaged[i] ? condition : kTriggerNone;
    }

    /**
     * What wheel {@code i} would engage on this tick, most specific reason first.
     *
     * <p>Through zero is a change of sign, not a large ratio. {@code |λ| > 1} on its own is routine
     * near {@code vFloor} — a wheel at 0.9 m/s against a 0.2 m/s reference is λ 2.33 with nothing
     * reversed — and the excess path already has that wheel; calling it a reversal would only
     * mis-file it in the trigger census this build ships to collect. A zero reference is not a
     * reversal either: there is no sign for the wheel to have crossed.
     */
    private int condition(int i, Limits limits) {
        if (lambda[i] * Math.signum(vRef[i]) < -1.0) return kTriggerReverse;
        if (Math.abs(excess[i]) > limits.dvHi()) return kTriggerExcess;
        return cmTrigger ? kTriggerCommon : kTriggerNone;
    }

    /**
     * Whether the wheels' twist has changed faster than {@code cmAccel} over the common-mode
     * window. False until the window has filled: a startup with no history is not an event.
     */
    private boolean commonMode(ChassisSpeeds fk, Limits limits) {
        int window = histFkX.length - 1;
        int slot = (int) (ticks % histFkX.length);
        int oldest = (slot + 1) % histFkX.length;
        double accel =
                ticks >= window
                        ? Math.hypot(
                                        fk.vxMetersPerSecond - histFkX[oldest],
                                        fk.vyMetersPerSecond - histFkY[oldest])
                                / (window * dt)
                        : 0.0;
        histFkX[slot] = fk.vxMetersPerSecond;
        histFkY[slot] = fk.vyMetersPerSecond;
        return accel > limits.cmAccel();
    }

    /** How far a wheel is from pointing where it is commanded, radians, folded into [0, π/2]. */
    static double headingError(Rotation2d measured, Rotation2d commanded) {
        double error = Math.abs(commanded.minus(measured).getRadians());
        return error > Math.PI / 2 ? Math.PI - error : error;
    }

    /** The speed the P term should see: the reference on an engaged wheel, the wheel otherwise. */
    public double[] measuredForP() {
        return measuredForP;
    }

    /** Per wheel factor on the composed amps; 1 where nothing is engaged or the switch is off. */
    public double[] scale() {
        return scale;
    }

    /** Signed slip ratio per wheel; NaN where the inputs were not finite. */
    public double[] lambda() {
        return lambda;
    }

    /** Signed excess of each wheel over its reference, m/s. */
    public double[] excess() {
        return excess;
    }

    /** The reference speed per wheel, m/s along the heading. */
    public double[] reference() {
        return vRef;
    }

    public boolean[] engaged() {
        return engaged;
    }

    /** Whether each wheel's speed was worth comparing to the reference this tick. */
    public boolean[] judged() {
        return judged;
    }

    /** What holds each wheel this tick: 0 none, 1 excess, 2 through zero, 3 common mode. */
    public int[] trigger() {
        return trigger;
    }

    /**
     * Whether there is a reference worth judging against — this tick and the two before it. An
     * invalid one unjudges every wheel, so everything engaged releases and the scales ramp back to
     * 1 on the usual law rather than stepping.
     */
    public boolean refValid() {
        return refInvalidTicks == 0;
    }

    /** Whether the wheels' twist broke away as a whole this tick. */
    public boolean cmTrigger() {
        return cmTrigger;
    }

    /** False when a wheel was left alone this tick because its slip ratio was not finite. */
    public boolean inputsFinite() {
        return inputsFinite;
    }
}
