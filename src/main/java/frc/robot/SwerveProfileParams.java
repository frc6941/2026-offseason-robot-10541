package frc.robot;

import lib.ironpulse.swerve.ChassisProfile;
import lib.ntext.NTParameter;

@NTParameter(tableName = "Params/Swerve/Profile")
public final class SwerveProfileParams {
    // SI units. Retain 10541's speed cap and its effective module acceleration cap.
    static final double maxLinearVelocity = 4.0;
    static final double maxLinearAccel = 15.0;
    static final double maxLinearJerk = ChassisProfile.kDefaultMaxLinearJerk;
    static final double maxBrakeAccel = 25.0;
    static final double maxBrakeJerk = ChassisProfile.kDefaultMaxBrakeJerk;
    // The profile also bounds angular velocity by module free speed / lever arm.
    static final double maxAngularVelocity = Math.toRadians(1000.0);
    static final double maxAngularAccel = Math.toRadians(5000.0);
    static final double maxAngularJerk = ChassisProfile.kDefaultMaxAngularJerk;
}
