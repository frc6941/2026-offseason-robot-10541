package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation3d;
import lib.ironpulse.utils.Logging;

public final class RobotConstants {
    // Mechanism geometry (robot frame: +X forward, +Y left, +Z up)
    // TODO: match HOOD_PIVOT to CAD. Single source of truth for the hood joint origin
    // (RobotMechanism3d 3D model) and the shooter muzzle release point (projectile viz).
    public static final Translation3d HOOD_PIVOT = new Translation3d(-0.2579, 0.0, 0.47525);

    public static final boolean HAS_INTAKER_IO = false;
    public static final boolean HAS_HOPPER_IO = true;
    public static final boolean HAS_SWERVE_IO = true;
    public static final boolean HAS_SHOOTER_IO = false;
    public static final boolean HAS_LED_IO = false;
    public static final boolean HAS_HOOD_IO = true;

    // NT live-tuning gate. When false, @NTParameter values never poll NetworkTables — they hold
    // their compile-time defaults and NTParameterRegistry.refresh() is a no-op, keeping the
    // per-loop NT JNI reads off the loop budget. Flip true to tune PID/params live from the
    // dashboard. Pushed into the ntext framework in Robot.robotInit(). See lib.ntext.
    public static final boolean ENABLE_NT_PARAMS = true;

    // CAN
    public static final String ROBORIO_CAN_BUS_NAME = "rio";
    public static final String CANIVORE_CAN_BUS_NAME = "10541Canivore0";
    public static final CANBus CANIVORE_CAN_BUS = new CANBus(CANIVORE_CAN_BUS_NAME);
    // Swerve lives on the CANivore; all other mechanisms (shooter, hood, hopper, intake) are on
    // the roboRIO bus.
    public static final CANBus ROBORIO_CAN_BUS = new CANBus(ROBORIO_CAN_BUS_NAME);

    // CAN_ID
    public static final int PIGEON_ID = 0;

    // Alliance flip
    public static boolean disableHAL = false;

    public static RobotConfig AUTO_ROBOT_CONFIG;

    static {
        try {
            AUTO_ROBOT_CONFIG = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            Logging.error("Constants", "Failed to load AUTO_ROBOT_CONFIG. %s", e.getMessage());
        }
    }

    // Robot Periodic
    public static final double LOOPER_DT = 0.02;

    private RobotConstants() {}
}
