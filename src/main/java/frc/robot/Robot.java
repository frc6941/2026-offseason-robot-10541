// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import lib.ironpulse.utils.LoggedTracer;
import lib.ironpulse.utils.PhoenixUtils;
import lib.ntext.NTParameterRegistry;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */

// The default command-based template uses TimedRobot. However, when we are using Advantagekit, we
// want to use LoggedRobot instead.
public class Robot extends LoggedRobot {
    // Keep the full AdvantageKit pipeline off the roboRIO while diagnosing loop overruns. Merely
    // omitting NT4Publisher is not enough: Logger.start() still clones the full log table each
    // loop.
    private static final boolean ENABLE_REAL_NT4_LOGGING = true;
    private static final boolean ENABLE_REAL_WPILOG_LOGGING = false;
    private static final long LOOP_WARNING_MICROS = 40_000;
    private static final long LOOP_WARNING_INTERVAL_MICROS = 1_000_000;

    private Command autonomousCommand;
    private final RobotContainer robotContainer;
    private long lastLoopWarningMicros = 0;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        // Push the NT live-tuning gate into the ntext framework once, before any refresh() runs.
        NTParameterRegistry.setEnabled(RobotConstants.ENABLE_NT_PARAMS);

        // AdvantageKit logger — sends data to AdvantageScope and writes .wpilog files
        boolean enableNt4 = RobotBase.isSimulation() || ENABLE_REAL_NT4_LOGGING;
        boolean enableWpilog = RobotBase.isReal() && ENABLE_REAL_WPILOG_LOGGING;

        if (enableNt4) {
            Logger.addDataReceiver(new NT4Publisher());
        }
        if (enableWpilog) {
            Logger.addDataReceiver(new WPILOGWriter());
        }
        if (enableNt4 || enableWpilog) {
            Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
            Logger.start();
        }
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Reset the shared stopwatch at the very top of the loop so each subsystem's
        // LoggedTracer.record(...) reports time elapsed since here (otherwise the deltas chain from
        // an arbitrary point and the first sample is garbage).
        LoggedTracer.reset();

        long loopStart = RobotController.getFPGATime();

        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        // Refresh all registered Phoenix status signals (swerve modules + Pigeon) at the top of the
        // loop. The multi-bus lib registers signals but does NOT refresh them itself — without
        // this,
        // module/IMU readings stay stale (frozen pose, frozen yaw). See SwerveModuleIOMK5N ctor.
        PhoenixUtils.refreshAll();
        long phoenixEnd = RobotController.getFPGATime();

        CommandScheduler.getInstance().run();
        long schedulerEnd = RobotController.getFPGATime();

        // Live NT tuning is gated by RobotConstants.ENABLE_NT_PARAMS (pushed into the registry in
        // robotInit). When off, this is a no-op — no per-loop NT JNI reads on the loop budget.
        NTParameterRegistry.refresh();
        robotContainer.updateDashboard();
        long loopEnd = RobotController.getFPGATime();

        reportLoopOverrun(loopStart, phoenixEnd, schedulerEnd, loopEnd);
    }

    private void reportLoopOverrun(
            long loopStart, long phoenixEnd, long schedulerEnd, long loopEnd) {
        long totalMicros = loopEnd - loopStart;
        if (totalMicros < LOOP_WARNING_MICROS
                || loopEnd - lastLoopWarningMicros < LOOP_WARNING_INTERVAL_MICROS) {
            return;
        }

        lastLoopWarningMicros = loopEnd;
        DriverStation.reportWarning(
                String.format(
                        "Robot loop %.1f ms (Phoenix %.1f, Scheduler %.1f, Dashboard %.1f)",
                        totalMicros / 1000.0,
                        (phoenixEnd - loopStart) / 1000.0,
                        (schedulerEnd - phoenixEnd) / 1000.0,
                        (loopEnd - schedulerEnd) / 1000.0),
                false);

        if (schedulerEnd - phoenixEnd >= LOOP_WARNING_MICROS) {
            CommandScheduler.getInstance().printWatchdogEpochs();
        }
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();
        DriverStation.reportWarning(
                "Selected auto: " + robotContainer.getAutoSelectionSummary(), false);

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
