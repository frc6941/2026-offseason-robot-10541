package lib.ironpulse.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Robot-frame chassis velocity predicted from the IMU and corrected by consistent wheels.
 *
 * <p>Gravity and learned accelerometer bias are removed before integration. Each wheel inside the
 * innovation gate contributes a bounded correction; wheels outside it cannot contaminate the
 * reference used to judge their slip. Consistent wheel subsets can recover the estimate after a
 * time hold, subject to torque-signed vetoes. Missing or stale IMU data falls back to wheel
 * kinematics. Bias is learned at rest, including while disabled.
 */
public final class BodyVelocityEstimator {
    /** The knobs, read by the caller from {@code Params/Swerve/BodyVel}. */
    public record Limits(
            double gateMps,
            double seedTauS,
            double seedRateCap,
            double biasTauS,
            double imuStaleS,
            double plausibleMps) {}

    /**
     * One tick of the IMU in the robot frame: whether the gyro answered and how old its sample is
     * (s), then linear acceleration with gravity still in it (m/s²), the attitude that leaks it
     * (rad), and the rate the frame turns at (rad/s).
     */
    public record Imu(
            boolean connected,
            double latencyS,
            double ax,
            double ay,
            double pitchRad,
            double rollRad,
            double omega) {}

    /** How far a wheel may be from the predicted body, m/s, and still be allowed to correct it. */
    public static final double kDefaultGateMps = 0.75;

    public static final double kDefaultSeedTauS = 0.05;
    public static final double kDefaultSeedRateCap = 4.0;
    public static final double kDefaultBiasTauS = 2.0;

    /** Sample age, s, past which the gyro is not answering even if it says it is connected. */
    public static final double kDefaultImuStaleS = 0.05;

    /**
     * Body speed, m/s, past which the state is not a measurement of anything. The modules' free
     * speed is 5.0 and the profile's cap 4.75, so a chassis the wheels could not have driven to is
     * an integrated sensor fault — a wrong accelerometer frame, an attitude error, a dead axis.
     */
    public static final double kDefaultPlausibleMps = 5.25;

    /** Gravity, m/s²: what pitch and roll leak into a robot-frame accelerometer. */
    static final double kG = 9.80665;

    /** Wheel speed under which a module counts as stopped, m/s. */
    static final double kRestSpeed = 0.05;

    /** Seconds every wheel must be stopped before the bias may be tracked. */
    static final double kRestHoldS = 0.5;

    /** Residual against their own fitted body velocity, m/s, under which wheels are one body. */
    static final double kSnapConsistentMps = 0.5;

    /**
     * Wheels this many must agree before their word can move the state outright. Two is enough
     * because the yaw rate is known: each wheel states the whole body velocity on its own, so two
     * of them are four equations in two unknowns rather than an under-determined twist.
     */
    static final int kSnapConsistentWheels = 2;

    /** Seconds those wheels must all say the same thing before the snap fires. */
    static final double kSnapHoldS = 0.05;

    /**
     * Heading error, radians, past which a wheel cannot veto a snap.
     *
     * <p>Looser than the feed gate's 10°, because the two ask different questions. Feeding needs
     * the wheel's heading to be accurate enough to correct a body velocity along; vetoing only
     * needs its <i>speed</i> to be impossible, and a wheel 25° off still measures a speed. Tying
     * the veto to the feed test cost it exactly when it was needed: a module 10.1° off during a
     * three-wheel lock was unjudgeable, so it could not feed, so it could not veto, and the locked
     * wheels took the reference to zero at 2 m/s.
     */
    static final double kVetoHeadingErr = Math.toRadians(30.0);

    /**
     * Disagreement with the candidates' fit, m/s, at which a wheel outside the subset vetoes.
     *
     * <p>Not {@link #kSnapConsistentMps} itself, so that every voter faces the same bar. A wheel on
     * its commanded heading becomes a candidate first, and {@link #subsetAgrees}'s drop-one step
     * only discards it once its residual passes {@code kSnapConsistentMps} — measured against a fit
     * that still contains it, so it reads {@code (n−1)/n} of the wheel's real disagreement, three
     * quarters of it on four wheels. Its effective threshold to become a vetoer is therefore {@code
     * kSnapConsistentMps / 0.75}. A wheel 10–30° off its command never becomes a candidate and
     * votes directly, so measuring it at the raw 0.5 would have it veto on a third less
     * disagreement than its on-heading neighbour, for no physical reason at all.
     */
    static final double kVetoMps = kSnapConsistentMps / 0.75;

    private final int moduleCount;
    private final double dt;
    private final double[] rx;
    private final double[] ry;
    private final int snapHoldTicks;

    /** Reused per-tick buffers: what each wheel says, and whether it was allowed to say it. */
    private final double[] innovations;

    private final boolean[] fed;
    private final boolean[] judgeable;
    private final boolean[] consistent;
    private final double[] bodyFit = new double[2];

    private double vx;
    private double vy;
    private double ax;
    private double ay;
    private double biasX;
    private double biasY;
    private boolean biasSeeded;
    private double horizonS;
    private double innovation;
    private boolean valid = true;
    private int feeding;
    private int judgeableWheels;
    private int snaps;
    private int snapTicks;
    private int plausibilitySnaps;
    private int restTicks;
    private int steerBlindTicks;

    public BodyVelocityEstimator(Translation2d[] moduleLocations, double dtS) {
        moduleCount = moduleLocations.length;
        dt = dtS;
        rx = new double[moduleCount];
        ry = new double[moduleCount];
        for (int i = 0; i < moduleCount; i++) {
            rx[i] = moduleLocations[i].getX();
            ry[i] = moduleLocations[i].getY();
        }
        innovations = new double[moduleCount];
        fed = new boolean[moduleCount];
        judgeable = new boolean[moduleCount];
        consistent = new boolean[moduleCount];
        snapHoldTicks = Math.max(1, (int) Math.round(kSnapHoldS / dt));
    }

    /**
     * One tick: predict from the IMU, then let every wheel inside the gate correct it.
     *
     * @param fk the wheels' four-wheel least-squares twist, robot frame
     * @param measuredMps per module measured drive speed, m/s, signed along the heading
     * @param headings per module measured heading
     * @param headingsCmd per module commanded heading — how far from measuring the body's own
     *     direction each wheel is
     * @param ampsCmdPrev per module composed drive current before the governor's scale, from the
     *     previous tick — only its sign is read, to tell which side of a fit a wheel could
     *     physically be on
     * @param steerCoupled whether modules still turning withheld part of the last step
     * @param imu this tick's robot-frame IMU sample
     * @param limits the live knobs
     */
    public void update(
            ChassisSpeeds fk,
            double[] measuredMps,
            Rotation2d[] headings,
            Rotation2d[] headingsCmd,
            double[] ampsCmdPrev,
            boolean steerCoupled,
            Imu imu,
            Limits limits) {
        steerBlindTicks =
                steerCoupled ? TractionGovernor.kBlindTicks : Math.max(0, steerBlindTicks - 1);
        valid = imu.connected() && imu.latencyS() < limits.imuStaleS();
        if (!valid) {
            holdOnWheels(fk);
            return;
        }

        double aRawX = imu.ax() - kG * Math.sin(imu.pitchRad());
        double aRawY = imu.ay() + kG * Math.sin(imu.rollRad());

        boolean rest = trackRest(measuredMps);
        if (rest && Double.isFinite(aRawX) && Double.isFinite(aRawY)) {
            if (biasSeeded) {
                biasX += (aRawX - biasX) * dt / limits.biasTauS();
                biasY += (aRawY - biasY) * dt / limits.biasTauS();
            } else {
                biasX = aRawX;
                biasY = aRawY;
                biasSeeded = true;
            }
        }
        ax = aRawX - biasX;
        ay = aRawY - biasY;

        // Predict: integrate in the frame the tick started in, then re-express in the one it ended
        // in. Everything below is the correction, and every wheel is judged against this state.
        double stepX = vx + ax * dt;
        double stepY = vy + ay * dt;
        double c = Math.cos(imu.omega() * dt);
        double s = Math.sin(imu.omega() * dt);
        vx = c * stepX + s * stepY;
        vy = c * stepY - s * stepX;

        correct(measuredMps, headings, headingsCmd, imu.omega(), limits);
        snap(measuredMps, headings, headingsCmd, ampsCmdPrev, imu.omega());

        // At rest the wheels are the truth outright, and a state that went non-finite has to have
        // somewhere to come back from.
        if (rest || !Double.isFinite(vx) || !Double.isFinite(vy)) {
            vx = fk.vxMetersPerSecond;
            vy = fk.vyMetersPerSecond;
            horizonS = 0.0;
        }
        bound(fk, limits);
    }

    /**
     * The last line under the integration: a body speed the wheels could not have produced.
     *
     * <p>Everything above corrects the state from the wheels, and every one of those paths needs
     * the wheels to be near it — the feed gate is 0.75 m/s wide and the snap wants two wheels
     * agreeing. An acceleration error larger than {@code seedRateCap} outruns all of them: 4.5 m/s²
     * of bias, which is a 24° attitude error or a wrong accelerometer frame, integrates to +444 m/s
     * in a hundred seconds with nothing able to pull it back. Past {@code plausibleMps} the state
     * has stopped being a measurement, so the wheels take it back outright however far away they
     * are, and {@link #plausibilitySnaps()} counts it — a bag showing any of these is reporting a
     * broken IMU, not a tuning problem.
     */
    private void bound(ChassisSpeeds fk, Limits limits) {
        if (Math.hypot(vx, vy) <= limits.plausibleMps()) return;
        boolean wheelsReadable =
                Double.isFinite(fk.vxMetersPerSecond) && Double.isFinite(fk.vyMetersPerSecond);
        // Always act: leaving the state alone because the wheels are past the bound too is how a
        // 4.5 m/s² bias with two over-reading encoders reached 436 m/s. Wheels that cannot be read
        // at all leave nothing to fall back to, and zero is nearer the truth than a runaway.
        vx = wheelsReadable ? fk.vxMetersPerSecond : 0.0;
        vy = wheelsReadable ? fk.vyMetersPerSecond : 0.0;
        // Count only when the wheels were inside the bound and could therefore be believed: that
        // is the case where the IMU alone was wrong, which is what this counter is for. A chassis
        // at the velocity cap with a common slip has both sources saying the robot is fast.
        if (!wheelsReadable
                || Math.hypot(fk.vxMetersPerSecond, fk.vyMetersPerSecond) <= limits.plausibleMps())
            plausibilitySnaps++;
        snapTicks = 0;
    }

    /**
     * The measurement update, one wheel at a time.
     *
     * <p>A wheel's innovation is what it measures minus what the predicted body would be doing
     * under it — no lead term, because this is a correction and not a prediction. Inside {@code
     * gateMps} the wheel pulls the state along its own heading at {@code 1/seedTauS}; outside it
     * the wheel says nothing at all, which is how a slipping wheel excludes itself from the
     * reference it is about to be judged against without anyone having to decide that it is
     * slipping. The tick's whole correction is bounded by {@code seedRateCap} as a rate, so even
     * four wheels agreeing on something wrong cannot step the reference.
     */
    private void correct(
            double[] measuredMps,
            Rotation2d[] headings,
            Rotation2d[] headingsCmd,
            double omega,
            Limits limits) {
        double correctX = 0.0;
        double correctY = 0.0;
        feeding = 0;
        judgeableWheels = 0;
        innovation = 0.0;
        for (int i = 0; i < moduleCount; i++) {
            double cos = headings[i].getCos();
            double sin = headings[i].getSin();
            double reference = (vx - omega * ry[i]) * cos + (vy + omega * rx[i]) * sin;
            innovations[i] = measuredMps[i] - reference;
            // A wheel that read non-finite is not a wheel worth reading: it is neither judgeable,
            // nor a snap candidate, nor able to veto one, and it does not enter any count.
            judgeable[i] =
                    Double.isFinite(innovations[i])
                            && steerBlindTicks == 0
                            && TractionGovernor.headingError(headings[i], headingsCmd[i])
                                    < TractionGovernor.kJudgeHeadingErr;
            fed[i] = judgeable[i] && Math.abs(innovations[i]) < limits.gateMps();
            // Reported for every wheel worth reading, not only the ones inside the gate: the tick
            // where nothing feeds is exactly the tick where how far out they are matters most. A
            // wheel that read NaN is not a large innovation, it is no innovation.
            if (judgeable[i]) {
                judgeableWheels++;
                innovation = Math.max(innovation, Math.abs(innovations[i]));
            }
            if (!fed[i]) continue;
            correctX += dt / limits.seedTauS() * innovations[i] * cos;
            correctY += dt / limits.seedTauS() * innovations[i] * sin;
            feeding++;
        }
        double magnitude = Math.hypot(correctX, correctY);
        double cap = limits.seedRateCap() * dt;
        if (magnitude > cap) {
            correctX *= cap / magnitude;
            correctY *= cap / magnitude;
        }
        vx += correctX;
        vy += correctY;
        if (feeding >= 2) horizonS = 0.0;
        else horizonS += dt;
    }

    /**
     * The one thing the wheels still do as a group: agree with each other, and disagree with the
     * state, hard enough and long enough to move it outright.
     *
     * <p>Two or more judgeable wheels whose residuals against {@link #subsetAgrees their own fitted
     * body velocity} are inside {@link #kSnapConsistentMps} are one rigid body as far as they are
     * concerned. If all of them have sat <i>outside the feed gate</i> for {@link #kSnapHoldS}, and
     * no readable wheel outside the subset {@link #vetoed contradicts them}, the state is wrong:
     * the gate is the whole definition of a wheel worth believing, so wheels that agree with each
     * other and are all beyond it have nothing left to say to it one at a time.
     *
     * <p>Sharing its threshold with the gate is what closes the window between them. A snap
     * threshold above {@code gateMps} leaves reference errors in between absorbing — no wheel near
     * enough to feed, no gap wide enough to snap — which on the 08-26 build was [0.75, 1.0] m/s,
     * cleared only by a full stop or an IMU dropout, and cost about a quarter of the braking
     * current on all four wheels while it lasted.
     */
    private void snap(
            double[] measuredMps,
            Rotation2d[] headings,
            Rotation2d[] headingsCmd,
            double[] ampsCmdPrev,
            double omega) {
        int candidates = 0;
        for (int i = 0; i < moduleCount; i++) {
            consistent[i] = judgeable[i] && !fed[i];
            if (consistent[i]) candidates++;
        }
        if (candidates < kSnapConsistentWheels
                || !subsetAgrees(measuredMps, headings, omega)
                || vetoed(measuredMps, headings, headingsCmd, ampsCmdPrev, omega)) {
            snapTicks = 0;
            return;
        }
        snapTicks++;
        if (snapTicks < snapHoldTicks) return;
        vx = bodyFit[0];
        vy = bodyFit[1];
        snaps++;
        snapTicks = 0;
    }

    /**
     * Whether a wheel that <i>is</i> being believed says the candidates cannot be right.
     *
     * <p>Wheels that agree with each other are not thereby correct: three wheels locking together
     * under braking agree perfectly, and left alone they would hand the reference their own stopped
     * speed and switch the ABS off at the moment it is needed. What separates that from three
     * wheels the IMU has drifted away from is not the wheels — it is the torque, and the feeding
     * wheel is the one still close enough to the state to be worth asking.
     *
     * <p>A driven wheel cannot be slower than the ground it is driving on, and a braked wheel
     * cannot be faster than the ground it is braking against. So a wheel outside the candidate
     * subset that sits on the <i>impossible</i> side of their fit — more than {@link #kVetoMps}
     * slow under positive current, or that far fast under negative — is evidence that the
     * candidates are the ones slipping, and it vetoes the snap. On the possible side it is no
     * evidence either way and the snap stands: that is the case where the reference had already
     * been dragged onto spinning wheels and only one of them is still spinning. A wheel with no
     * current commanded has no side and vetoes nothing.
     *
     * <p>Every readable wheel outside the subset votes, not only the ones still feeding. Requiring
     * nearness to the state asked the wrong wheels: the state is the thing in doubt, so a wheel far
     * from it is the one most likely to be right. On 2026-08-27 a stop-and-reverse left the
     * reference at 2.55 m/s, three wheels spun to 4.3–4.7 on the launch and one stayed honest at
     * 1.0 — the honest wheel was 3.5 m/s from the reference, so under the old rule it had no vote,
     * the three consistent spinners were believed, and the reference went to 4.59 until the
     * plausibility bound caught it at 5.225. It is now the dropped outlier, and it vetoes.
     */
    private boolean vetoed(
            double[] measuredMps,
            Rotation2d[] headings,
            Rotation2d[] headingsCmd,
            double[] ampsCmdPrev,
            double omega) {
        for (int i = 0; i < moduleCount; i++) {
            // Every readable wheel that is not one of the candidates gets a vote: the ones still
            // feeding, the outlier the drop-one step discarded, and anything the feed's own heading
            // test excluded. Nearness to the state is deliberately NOT required — the state is the
            // thing in doubt, so asking only wheels that already agree with it is asking the wrong
            // wheels, and that is exactly how the 2026-08-27 reversal was lost.
            if (consistent[i]) continue;
            if (!Double.isFinite(measuredMps[i])) continue;
            if (!(TractionGovernor.headingError(headings[i], headingsCmd[i]) < kVetoHeadingErr))
                continue;
            if (!Double.isFinite(ampsCmdPrev[i])) continue;
            double torque = Math.signum(ampsCmdPrev[i]);
            if (torque == 0.0) continue;
            double residual = measuredMps[i] - fitted(i, headings[i], omega);
            if (torque > 0.0 && residual < -kVetoMps) return true;
            if (torque < 0.0 && residual > kVetoMps) return true;
        }
        return false;
    }

    /** What {@link #bodyFit} says wheel {@code i} should be reading, m/s along its heading. */
    private double fitted(int i, Rotation2d heading, double omega) {
        return (bodyFit[0] - omega * ry[i]) * heading.getCos()
                + (bodyFit[1] + omega * rx[i]) * heading.getSin();
    }

    /**
     * Whether the wheels flagged in {@link #consistent} fit one body velocity, leaving it in {@link
     * #bodyFit}.
     *
     * <p>Each candidate is measured against the velocity <i>the candidates themselves</i> fit, and
     * never against the four-wheel one. The four-wheel fit contains every wheel including the one
     * being doubted, so a single wheel sitting inside the gate of a <i>wrong</i> reference pushes
     * its partners' residuals up by about two thirds of the drift and makes the honest majority
     * look inconsistent — fail-closed, and it cost 12–50 % of braking current at low speed.
     *
     * <p>The yaw rate is known, so this is not a twist fit at all: subtracting {@code ω × r_i}
     * leaves every wheel stating the whole body velocity by itself, and the fit is their mean. Two
     * wheels are therefore well-posed, and four are better conditioned than the three-unknown
     * version ever was.
     *
     * <p>One outlier may be dropped and the fit retried. The dropped wheel's residual is measured
     * against a fit that still contains it, so it reads about {@code (n−1)/n} of its true distance
     * and a snap can land up to {@code kSnapConsistentMps/(n−1)} off the honest majority: 0.167 m/s
     * on four candidates, 0.5 m/s on two. The four-wheel figure is inside the governor's own {@code
     * dvHi}; the two-wheel one is not, and both are accepted.
     */
    private boolean subsetAgrees(double[] measuredMps, Rotation2d[] headings, double omega) {
        for (int attempt = 0; attempt < 2; attempt++) {
            int count = 0;
            for (int i = 0; i < moduleCount; i++) if (consistent[i]) count++;
            if (count < kSnapConsistentWheels || !subsetVelocity(measuredMps, headings, omega))
                return false;

            int worst = -1;
            double worstResidual = kSnapConsistentMps;
            for (int i = 0; i < moduleCount; i++) {
                if (!consistent[i]) continue;
                double residual = Math.abs(measuredMps[i] - fitted(i, headings[i], omega));
                if (!(residual <= worstResidual)) {
                    worst = i;
                    worstResidual = residual;
                }
            }
            if (worst < 0) return true;
            consistent[worst] = false;
        }
        return false;
    }

    /**
     * The body velocity the candidates agree on: each states it in full once {@code ω × r_i} is
     * taken off its own reading, so the fit is the mean of what they say.
     */
    private boolean subsetVelocity(double[] measuredMps, Rotation2d[] headings, double omega) {
        double sumX = 0.0;
        double sumY = 0.0;
        int count = 0;
        for (int i = 0; i < moduleCount; i++) {
            if (!consistent[i]) continue;
            sumX += measuredMps[i] * headings[i].getCos() + omega * ry[i];
            sumY += measuredMps[i] * headings[i].getSin() - omega * rx[i];
            count++;
        }
        if (count == 0) return false;
        bodyFit[0] = sumX / count;
        bodyFit[1] = sumY / count;
        return Double.isFinite(bodyFit[0]) && Double.isFinite(bodyFit[1]);
    }

    /**
     * What the state does with no IMU behind it: sit on the wheels' own twist.
     *
     * <p>A Pigeon that has stopped answering reads zero, not nothing, so integrating it integrates
     * a lie — and the lie compounds, because a zero yaw rate also stops the state being carried
     * through the turn. On 2026-08-26 that cost 0.92 s of a −6 rad/s spin and came back 4.41 m/s
     * out. Holding on the wheels is not a measurement of the chassis either, but it is bounded, it
     * is what the governor is about to be told not to trust anyway, and it leaves no drift to
     * unwind when the gyro returns. The rest counter goes with it: a stopped accelerometer at a
     * standstill is exactly the shape of a bias measurement while being none.
     */
    private void holdOnWheels(ChassisSpeeds fk) {
        // With no IMU this returns early, so the plausibility bound never runs and nothing else
        // would catch a non-finite twist: the state would publish NaN for as long as the gyro
        // stayed away. Zero is not the chassis either, but it is a number.
        boolean wheelsReadable =
                Double.isFinite(fk.vxMetersPerSecond) && Double.isFinite(fk.vyMetersPerSecond);
        vx = wheelsReadable ? fk.vxMetersPerSecond : 0.0;
        vy = wheelsReadable ? fk.vyMetersPerSecond : 0.0;
        ax = 0.0;
        ay = 0.0;
        innovation = 0.0;
        feeding = 0;
        judgeableWheels = 0;
        java.util.Arrays.fill(fed, false);
        restTicks = 0;
        snapTicks = 0;
        horizonS += dt;
    }

    /** True once every wheel has been stopped for {@link #kRestHoldS}. */
    private boolean trackRest(double[] measuredMps) {
        boolean stopped = true;
        for (int i = 0; i < moduleCount; i++)
            if (!(Math.abs(measuredMps[i]) < kRestSpeed)) stopped = false;
        restTicks = stopped ? restTicks + 1 : 0;
        return restTicks >= Math.max(1, (int) Math.round(kRestHoldS / dt));
    }

    /** Body velocity along the robot's x, m/s. */
    public double vx() {
        return vx;
    }

    /** Body velocity along the robot's y, m/s. */
    public double vy() {
        return vy;
    }

    /** Body acceleration along the robot's x, m/s², gravity and bias removed. */
    public double ax() {
        return ax;
    }

    /** Body acceleration along the robot's y, m/s², gravity and bias removed. */
    public double ay() {
        return ay;
    }

    /** Whether the IMU answered this tick with a fresh sample. False means there is no estimate. */
    public boolean valid() {
        return valid;
    }

    /** Which wheels were inside the gate and corrected the state this tick. */
    public boolean[] fed() {
        return fed;
    }

    /** How many of them there were. */
    public int feeding() {
        return feeding;
    }

    /**
     * How many wheels were worth reading at all this tick. With {@link #feeding()} it separates the
     * two ways a tick can have nothing to say: no wheel judgeable, or every wheel dead on.
     */
    public int judgeableWheels() {
        return judgeableWheels;
    }

    /** How many times agreeing wheels have moved the state outright. */
    public int snaps() {
        return snaps;
    }

    /**
     * How many times the state has been thrown out for claiming a speed the wheels cannot reach.
     */
    public int plausibilitySnaps() {
        return plausibilitySnaps;
    }

    /** Seconds since two wheels last fed — how long the state has run mostly on the IMU. */
    public double horizonS() {
        return horizonS;
    }

    /**
     * The largest gap any wheel worth reading is reporting, m/s — feeding or not, so this still
     * says something on the ticks where nothing feeds. Wheels that read non-finite are not
     * judgeable at all, so they never reach it. Zero means every judgeable wheel is dead on, or
     * that {@link #judgeableWheels()} is itself zero and there were none; {@link #feeding()} says
     * how many of them were close enough to be believed.
     */
    public double innovation() {
        return innovation;
    }

    /** Tracked accelerometer bias along the robot's x, m/s². */
    public double biasX() {
        return biasX;
    }

    /** Tracked accelerometer bias along the robot's y, m/s². */
    public double biasY() {
        return biasY;
    }
}
