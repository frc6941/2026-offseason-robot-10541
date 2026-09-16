package frc.robot;

import lib.ironpulse.swerve.TractionGovernor;
import lib.ntext.NTParameter;

@NTParameter(tableName = "Params/Swerve/Traction")
public final class SwerveTractionParams {
    static final boolean enabled = true;
    static final double lambdaStar = TractionGovernor.kDefaultLambdaStar;
    static final double dvHi = TractionGovernor.kDefaultDvHi;
    static final double kP = TractionGovernor.kDefaultKP;
    static final double scaleMin = TractionGovernor.kDefaultScaleMin;
    static final double releaseS = TractionGovernor.kDefaultReleaseS;
    static final double vFloor = TractionGovernor.kDefaultVFloor;
    static final double cmAccel = TractionGovernor.kDefaultCmAccel;
    static final double leadS = TractionGovernor.kDefaultLeadS;
}
