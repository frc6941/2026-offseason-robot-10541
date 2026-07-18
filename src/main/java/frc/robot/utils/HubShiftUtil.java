package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

/** Computes the current 2026 Hub phase and the time until its next state change. */
public final class HubShiftUtil {
    private static final double AUTO_DURATION_SECONDS = 20.0;
    private static final double TELEOP_DURATION_SECONDS = 140.0;
    private static final Timer modeTimer = new Timer();

    private HubShiftUtil() {}

    /** Restart the fallback clock whenever autonomous or teleop begins. */
    public static void initialize() {
        modeTimer.restart();
    }

    public static ShiftInfo getShiftInfo() {
        if (DriverStation.isAutonomousEnabled()) {
            double remaining = remainingModeTime(AUTO_DURATION_SECONDS);
            return new ShiftInfo(HubPhase.AUTO, remaining, true);
        }
        if (!DriverStation.isTeleopEnabled()) {
            return new ShiftInfo(HubPhase.DISABLED, 0.0, false);
        }

        double matchRemaining = remainingModeTime(TELEOP_DURATION_SECONDS);
        HubPhase phase;
        double phaseRemaining;
        if (matchRemaining > 130.0) {
            phase = HubPhase.TRANSITION;
            phaseRemaining = matchRemaining - 130.0;
        } else if (matchRemaining > 105.0) {
            phase = HubPhase.SHIFT1;
            phaseRemaining = matchRemaining - 105.0;
        } else if (matchRemaining > 80.0) {
            phase = HubPhase.SHIFT2;
            phaseRemaining = matchRemaining - 80.0;
        } else if (matchRemaining > 55.0) {
            phase = HubPhase.SHIFT3;
            phaseRemaining = matchRemaining - 55.0;
        } else if (matchRemaining > 30.0) {
            phase = HubPhase.SHIFT4;
            phaseRemaining = matchRemaining - 30.0;
        } else {
            phase = HubPhase.ENDGAME;
            phaseRemaining = matchRemaining;
        }

        return new ShiftInfo(phase, Math.max(0.0, phaseRemaining), isHubActive(phase));
    }

    private static double remainingModeTime(double modeDurationSeconds) {
        double dsMatchTime = DriverStation.getMatchTime();
        double remaining = dsMatchTime >= 0.0 ? dsMatchTime : modeDurationSeconds - modeTimer.get();
        return MathUtil.clamp(remaining, 0.0, modeDurationSeconds);
    }

    private static boolean isHubActive(HubPhase phase) {
        if (phase == HubPhase.AUTO || phase == HubPhase.TRANSITION || phase == HubPhase.ENDGAME) {
            return true;
        }
        if (phase == HubPhase.DISABLED) {
            return false;
        }

        String gameData = DriverStation.getGameSpecificMessage();
        var alliance = DriverStation.getAlliance();
        if (gameData.isEmpty() || alliance.isEmpty()) {
            // Without field data, stay permissive instead of incorrectly blocking a shot.
            return true;
        }

        boolean redInactiveFirst;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                return true;
            }
        }

        boolean shift1Active =
                alliance.get() == Alliance.Red ? !redInactiveFirst : redInactiveFirst;
        return switch (phase) {
            case SHIFT1, SHIFT3 -> shift1Active;
            case SHIFT2, SHIFT4 -> !shift1Active;
            case AUTO, TRANSITION, ENDGAME -> true;
            case DISABLED -> false;
        };
    }

    public enum HubPhase {
        AUTO,
        TRANSITION,
        SHIFT1,
        SHIFT2,
        SHIFT3,
        SHIFT4,
        ENDGAME,
        DISABLED
    }

    public record ShiftInfo(HubPhase phase, double remainingTimeSeconds, boolean active) {}
}
