package lib.ironpulse.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * One tick's chassis reference: the twist, the module states it maps to, and the acceleration the
 * profile planned it with — robot frame, so the composer never has to differentiate the twist.
 */
public record SwerveSetpoint(
        ChassisSpeeds chassisSpeeds, SwerveModuleState[] moduleStates, ChassisAccel accel) {
    public SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] moduleStates) {
        this(chassisSpeeds, moduleStates, new ChassisAccel(0.0, 0.0, 0.0));
    }
}
