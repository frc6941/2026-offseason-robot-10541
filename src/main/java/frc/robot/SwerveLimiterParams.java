package frc.robot;

import lib.ironpulse.swerve.SwerveLimiter;
import lib.ntext.NTParameter;

@NTParameter(tableName = "Params/Swerve/Limiter")
public final class SwerveLimiterParams {
    static final boolean enabled = true;

    /** Supply-current budget in amperes, shared with other loads if pack current is supplied. */
    static final double iBudgetAmps = SwerveLimiter.kDefaultIBudgetAmps;
}
