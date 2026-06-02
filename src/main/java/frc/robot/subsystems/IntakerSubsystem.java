package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakerSubsystem extends SubsystemBase {

    private final PWMSparkMax rollerMotor = new PWMSparkMax(0);

    public IntakerSubsystem() {}

    public void runRoller(double speed) {
        rollerMotor.set(speed);
    }

    public void stopRoller() {
        rollerMotor.set(0);
    }

    public Command intake() {
        return run(() -> runRoller(0.7));
    }

    public Command outtake() {
        return run(() -> runRoller(-0.7));
    }

    public Command stop() {
        return runOnce(this::stopRoller);
    }

    @Override
    public void periodic() {}
}
