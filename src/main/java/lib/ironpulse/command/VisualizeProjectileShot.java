package lib.ironpulse.command;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Visualizes a ballistic projectile path given explicit shot parameters.
 *
 * <p>This command is intentionally "container-free": it does not query any subsystems or robot
 * singletons.
 *
 * <p>Model (current): gravity-only, integrated with a fixed timestep.
 */
public class VisualizeProjectileShot extends Command {
    private static final String kTag = "Commands/VisualizeProjectileShot";

    private static final double kDtSec = 0.06;
    private static final double kMaxTimeSec = 2.0;
    private static final double kStopHeightM = 0.0;
    private static final double kGravityMps2 = 9.81;

    private final Supplier<Pose3d> releasePoseWorldSupplier;
    private final Supplier<Rotation2d> yawWorldSupplier;
    private final Supplier<Rotation2d> pitchSupplier;
    private final DoubleSupplier muzzleSpeedMpsSupplier;
    private final Supplier<Translation2d> addedVelocityWorldMpsSupplier;
    private final boolean useGravity;

    private boolean isFinished;

    /**
     * All frames are field/world frame.
     *
     * @param releasePoseWorldSupplier World-frame pose of the projectile at release (position is
     *     used).
     * @param yawWorldSupplier World-frame yaw (CCW+).
     * @param pitchSupplier Pitch relative to horizontal, up-positive.
     * @param muzzleSpeedMpsSupplier Initial speed at release, in m/s.
     * @param addedVelocityWorldMpsSupplier Additional world-frame XY velocity to add (e.g., chassis
     *     field velocity). Return {@code new Translation2d()} or {@code null} if unused.
     * @param useGravity If true, apply gravity; if false, draw a straight-line shot.
     */
    public VisualizeProjectileShot(
            Supplier<Pose3d> releasePoseWorldSupplier,
            Supplier<Rotation2d> yawWorldSupplier,
            Supplier<Rotation2d> pitchSupplier,
            DoubleSupplier muzzleSpeedMpsSupplier,
            Supplier<Translation2d> addedVelocityWorldMpsSupplier,
            boolean useGravity) {
        this.releasePoseWorldSupplier =
                Objects.requireNonNull(releasePoseWorldSupplier, "releasePoseWorldSupplier");
        this.yawWorldSupplier = Objects.requireNonNull(yawWorldSupplier, "yawWorldSupplier");
        this.pitchSupplier = Objects.requireNonNull(pitchSupplier, "pitchSupplier");
        this.muzzleSpeedMpsSupplier =
                Objects.requireNonNull(muzzleSpeedMpsSupplier, "muzzleSpeedMpsSupplier");
        this.addedVelocityWorldMpsSupplier =
                (addedVelocityWorldMpsSupplier != null)
                        ? addedVelocityWorldMpsSupplier
                        : Translation2d::new;
        this.useGravity = useGravity;
    }

    public VisualizeProjectileShot(
            Supplier<Pose3d> releasePoseWorldSupplier,
            Supplier<Rotation2d> yawWorldSupplier,
            Supplier<Rotation2d> pitchSupplier,
            DoubleSupplier muzzleSpeedMpsSupplier,
            boolean useGravity) {
        this(
                releasePoseWorldSupplier,
                yawWorldSupplier,
                pitchSupplier,
                muzzleSpeedMpsSupplier,
                Translation2d::new,
                useGravity);
    }

    public VisualizeProjectileShot(
            Pose3d releasePoseWorld,
            Rotation2d yawWorld,
            Rotation2d pitch,
            double muzzleSpeedMps,
            boolean useGravity) {
        this(
                () -> releasePoseWorld,
                () -> yawWorld,
                () -> pitch,
                () -> muzzleSpeedMps,
                () -> new Translation2d(),
                useGravity);
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {
        Pose3d releasePoseWorldSnapshot =
                Objects.requireNonNull(
                        releasePoseWorldSupplier.get(), "releasePoseWorldSupplier.get()");
        Rotation2d yawWorldSnapshot =
                Objects.requireNonNull(yawWorldSupplier.get(), "yawWorldSupplier.get()");
        Rotation2d pitchSnapshot =
                Objects.requireNonNull(pitchSupplier.get(), "pitchSupplier.get()");
        double muzzleSpeedMps =
                Objects.requireNonNull(
                        muzzleSpeedMpsSupplier.getAsDouble(), "muzzleSpeedMpsSupplier.get()");
        Translation2d vAdded = addedVelocityWorldMpsSupplier.get();

        logPath(
                releasePoseWorldSnapshot,
                yawWorldSnapshot,
                pitchSnapshot,
                muzzleSpeedMps,
                vAdded,
                useGravity,
                "");
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput(kTag + "/muzzleSpeedMps", 0.0);
        Logger.recordOutput(kTag + "/addedVelocityWorldMps", new Translation2d());
        Logger.recordOutput(kTag + "/useGravity", false);
        Logger.recordOutput(kTag + "/pathWorld", new Pose3d[0]);
    }

    public static void logPath(
            Pose3d releasePoseWorld,
            Rotation2d yawWorld,
            Rotation2d pitch,
            double muzzleSpeedMps,
            Translation2d addedVelocityWorldMps,
            boolean useGravity,
            String logTag) {
        Logger.recordOutput(kTag + "/pathWorld" + logTag, new Pose3d[0]);
        Pose3d releasePoseWorldSnapshot =
                Objects.requireNonNull(releasePoseWorld, "releasePoseWorld");
        Rotation2d yawWorldSnapshot = Objects.requireNonNull(yawWorld, "yawWorld");
        Rotation2d pitchSnapshot = Objects.requireNonNull(pitch, "pitch");
        Translation2d addedVelocityWorldMpsSnapshot =
                (addedVelocityWorldMps != null) ? addedVelocityWorldMps : new Translation2d();

        Pose3d[] pathWorld =
                simulateGravityOnlyPath(
                        releasePoseWorldSnapshot,
                        yawWorldSnapshot,
                        pitchSnapshot,
                        muzzleSpeedMps,
                        addedVelocityWorldMpsSnapshot,
                        useGravity);

        Logger.recordOutput(kTag + "/muzzleSpeedMps", muzzleSpeedMps);
        Logger.recordOutput(kTag + "/addedVelocityWorldMps", addedVelocityWorldMpsSnapshot);
        Logger.recordOutput(kTag + "/useGravity", useGravity);
        Logger.recordOutput(kTag + "/pathWorld" + logTag, pathWorld);
    }

    /**
     * Gravity-only projectile simulation.
     *
     * <p>Frame conventions used here:
     *
     * <ul>
     *   <li>X/Y are field axes in the normal WPILib field coordinate system.
     *   <li>Z is up.
     *   <li>{@code yawWorld} is a field-relative yaw (CCW+).
     *   <li>{@code pitch} is up-positive from horizontal.
     * </ul>
     */
    public static Pose3d[] simulateGravityOnlyPath(
            Pose3d releasePoseWorld,
            Rotation2d yawWorld,
            Rotation2d pitch,
            double muzzleSpeedMps,
            Translation2d addedVelocityWorldMps,
            boolean useGravity) {
        double dtSec = kDtSec;
        double maxTimeSec = kMaxTimeSec;
        double stopHeightM = kStopHeightM;
        double gravityMps2 = kGravityMps2;

        Translation3d pos = releasePoseWorld.getTranslation();
        Translation3d vel =
                initialVelocityWorld(yawWorld, pitch, muzzleSpeedMps, addedVelocityWorldMps);

        Translation3d accel =
                useGravity
                        ? new Translation3d(0.0, 0.0, -Math.abs(gravityMps2))
                        : new Translation3d();

        List<Pose3d> samples = new ArrayList<>();
        samples.add(new Pose3d(pos, new Rotation3d()));

        int maxSteps = (int) Math.ceil(maxTimeSec / dtSec);
        for (int i = 0; i < maxSteps; i++) {
            // Semi-implicit Euler (stable enough for visualization).
            vel = vel.plus(accel.times(dtSec));
            pos = pos.plus(vel.times(dtSec));
            samples.add(new Pose3d(pos, new Rotation3d()));

            if (pos.getZ() <= stopHeightM) {
                break;
            }
        }

        return samples.toArray(Pose3d[]::new);
    }

    /**
     * Computes the field/world-frame initial projectile velocity from yaw/pitch/speed, plus any
     * additional field-relative velocity (e.g., chassis translation velocity).
     */
    public static Translation3d initialVelocityWorld(
            Rotation2d yawWorld,
            Rotation2d pitch,
            double muzzleSpeedMps,
            Translation2d addedVelocityWorldMps) {
        double yawRad = yawWorld.getRadians();
        double pitchRad = pitch.getRadians();

        double cosPitch = Math.cos(pitchRad);
        Translation3d vProjectile =
                new Translation3d(
                        muzzleSpeedMps * cosPitch * Math.cos(yawRad),
                        muzzleSpeedMps * cosPitch * Math.sin(yawRad),
                        muzzleSpeedMps * Math.sin(pitchRad));

        Translation3d vAdd3 =
                new Translation3d(addedVelocityWorldMps.getX(), addedVelocityWorldMps.getY(), 0.0);
        return vProjectile.plus(vAdd3);
    }
}
