package lib.ironpulse.indicator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class IndicatorSubsystem extends SubsystemBase {
    private final IndicatorIO io;
    private final IndicatorIOInputsAutoLogged inputs = new IndicatorIOInputsAutoLogged();
    private IndicatorIO.Patterns currentPattern = IndicatorIO.Patterns.NORMAL;
    @Getter private boolean outsideDefault = false;

    public IndicatorSubsystem(IndicatorIO io) {
        this.io = io;
    }

    public void setPattern(IndicatorIO.Patterns pattern) {
        currentPattern = pattern;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.setPattern(currentPattern);
        Logger.processInputs("Indicator", inputs);
        Logger.recordOutput("Indicator/Pattern", currentPattern.toString());
        SmartDashboard.putString("Indicator/Pattern", currentPattern.toString());
    }

    public void reset() {
        this.io.reset();
    }

    public Command indicate(IndicatorIO.Patterns pattern) {
        var cmd =
                Commands.sequence(
                                Commands.runOnce(
                                        () -> {
                                            outsideDefault = true;
                                            setPattern(pattern);
                                        },
                                        this),
                                Commands.waitUntil(() -> false))
                        .finallyDo(() -> outsideDefault = false)
                        .ignoringDisable(true);
        return cmd;
    }

    public Command indicateWithTimeout(IndicatorIO.Patterns pattern, double timeoutSeconds) {
        var cmd =
                Commands.sequence(
                                Commands.runOnce(
                                        () -> {
                                            outsideDefault = true;
                                            setPattern(pattern);
                                        }),
                                Commands.waitSeconds(timeoutSeconds),
                                Commands.runOnce(
                                        () -> {
                                            outsideDefault = false;
                                        }))
                        .finallyDo(() -> outsideDefault = false)
                        .ignoringDisable(true);
        return cmd;
    }
}
