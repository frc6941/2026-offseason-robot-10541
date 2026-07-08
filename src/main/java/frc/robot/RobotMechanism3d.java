package frc.robot;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import lib.ironpulse.io.MotorIO;
import lib.ironpulse.io.MotorInputsAutoLogged;
import lib.ironpulse.subsystem.position.PositionMotorSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Publishes the 3D pose of each articulated robot component so AdvantageScope can animate a CAD
 * model in sync with the log (à la 6328's {@code DarwinMechanism3d}). Visualization only — no
 * control logic.
 *
 * <p>The logged value is a {@code Pose3d[]} under {@code Mechanism3d/Measured}; AdvantageScope
 * binds component <i>i</i> of its robot config to index <i>i</i> of this array. The order is fixed
 * below. Each pose is robot-relative: the component's joint origin (a fixed mount {@link
 * Translation3d} from robot center) plus the live joint rotation.
 *
 * <p>TODO: TUNE FROM CAD — the mount translations and rotation axes/signs below are placeholders.
 * Export your robot model split into components, read each joint origin off the CAD, and confirm
 * which axis (and sign) each joint rotates about. WPILib robot frame: +X forward, +Y left, +Z up; a
 * hood/intake that pitches rotates about Y.
 */
public class RobotMechanism3d extends SubsystemBase {
    // ---- Component order (must match the AdvantageScope robot config) ----
    private static final int HOOD = 0;
    private static final int INTAKE = 1;
    private static final int COMPONENT_COUNT = 2;

    // TODO: tune joint origins (meters, relative to robot center) from CAD
    // ---- Joint origins relative to robot center (meters) ----
    // HOOD_PIVOT is shared with the shooter muzzle viz — see RobotConstants.HOOD_PIVOT.
    private static final Translation3d HOOD_PIVOT = RobotConstants.HOOD_PIVOT;
    private static final Translation3d INTAKE_PIVOT = new Translation3d(0.2829, 0.0, 0.20125);

    private final PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood;
    private final IntakerSubsystem intaker;

    public RobotMechanism3d(
            PositionMotorSubsystem<MotorInputsAutoLogged, MotorIO, Angle> hood,
            IntakerSubsystem intaker) {
        this.hood = hood;
        this.intaker = intaker;
    }

    @Override
    public void periodic() {
        double hoodRad = hood.getCurrPos().in(Radians);
        double intakeRad = intaker.getPivotAngle().in(Radians);

        Pose3d[] components = new Pose3d[COMPONENT_COUNT];
        // TODO: verify each joint's rotation axis and sign against CAD (both assumed pitch about
        // +Y)
        components[HOOD] = new Pose3d(HOOD_PIVOT, new Rotation3d(0.0, hoodRad, 0.0));
        components[INTAKE] = new Pose3d(INTAKE_PIVOT, new Rotation3d(0.0, intakeRad, 0.0));

        Logger.recordOutput("Mechanism3d/Measured", components);
    }
}
