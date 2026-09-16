package frc.robot;

import lib.ironpulse.swerve.BodyVelocityEstimator;
import lib.ntext.NTParameter;

@NTParameter(tableName = "Params/Swerve/BodyVel")
public final class SwerveBodyVelParams {
    static final double gateMps = BodyVelocityEstimator.kDefaultGateMps;
    static final double seedTauS = BodyVelocityEstimator.kDefaultSeedTauS;
    static final double seedRateCap = BodyVelocityEstimator.kDefaultSeedRateCap;
    static final double biasTauS = BodyVelocityEstimator.kDefaultBiasTauS;
    static final double imuStaleS = BodyVelocityEstimator.kDefaultImuStaleS;
    static final double plausibleMps = BodyVelocityEstimator.kDefaultPlausibleMps;
}
