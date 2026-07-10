package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.utils.AllianceFlipUtil;

/**
 * Default / fallback autonomous. The real routines are the PathPlanner-pathfinding autos built in
 * {@code AutoBuilder}/{@code AutoCommands} and selected via {@code AutoSelector}; this holds the
 * simple non-PathPlanner default (drive forward) used as the chooser's fallback — it still works
 * even if path generation fails.
 */
public final class DefaultAuto {
    private DefaultAuto() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Drive forward (1 m/s for 2 s) from center start — simplest possible auto.
     *
     * <p>The start pose is alliance-flipped, so on red the robot resets to the mirrored center
     * start facing into the field. The drive twist is robot-relative (see {@link Swerve#runTwist}),
     * so "forward" (+x) drives away from the wall for both alliances without flipping the velocity.
     */
    public static Command driveForward(Swerve swerve) {
        return Commands.sequence(
                SwerveCommands.reset(
                        swerve,
                        new Pose3d(
                                AllianceFlipUtil.apply(FieldConstants.StartPositions.BLUE_CENTER))),
                Commands.run(() -> swerve.runTwist(new ChassisSpeeds(1.0, 0.0, 0.0)), swerve)
                        .withTimeout(2.0)
                        .andThen(Commands.runOnce(swerve::runStop, swerve)));
    }
}
