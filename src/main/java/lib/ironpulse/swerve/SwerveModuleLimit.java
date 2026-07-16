package lib.ironpulse.swerve;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import lombok.Builder;

/** Enforces drive and steer limit at the module level. */
@Builder
public record SwerveModuleLimit(
        LinearVelocity maxDriveVelocity,
        LinearAcceleration maxDriveAcceleration,
        LinearAcceleration maxDriveDeceleration,
        AngularVelocity maxSteerAngularVelocity,
        AngularAcceleration maxSteerAngularAcceleration) {
    /**
     * Deceleration cap for a wheel that is slowing down (desired wheel speed &lt; previous wheel
     * speed). Falls back to {@link #maxDriveAcceleration()} when unset, preserving the original
     * symmetric behavior for any config that does not opt in to an asymmetric brake limit.
     */
    public LinearAcceleration maxDriveDecelerationOrAccel() {
        return maxDriveDeceleration != null ? maxDriveDeceleration : maxDriveAcceleration;
    }
}
