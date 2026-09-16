package frc.robot;

import lib.ironpulse.swerve.SwerveDriveComposer;
import lib.ntext.NTParameter;

@NTParameter(tableName = "Params/Swerve/Compose")
public final class SwerveDriveComposerParams {
    /** Uncalibrated on 10541: use its configured mass without the source robot's trim. */
    static final double scale = 1.0;

    /** P measurement low-pass corner in Hz; zero bypasses the filter. */
    static final double pFilterHz = SwerveDriveComposer.kDefaultPFilterHz;
}
