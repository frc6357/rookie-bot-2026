package frc.robot.bindings;

import static frc.robot.Ports.DriverPorts.kRBbutton;
import static frc.robot.Ports.DriverPorts.kRTrigger;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.robot.Konstants.DrumLauncherConstants;
import frc.robot.subsystems.drumlauncher.SK26DrumLauncher;
import frc.robot.subsystems.vision.SKVision;

/**
 * Button bindings for the drum launcher subsystem.
 *
 * <ul>
 *   <li><b>Right Trigger (held)</b> – spin the launcher at the RPM calculated
 *       from the vision-reported distance to the hub
 *       ({@link SK26DrumLauncher#runAtDistanceCommand}).</li>
 *   <li><b>Right Bumper (held)</b> – spin the launcher in shuttle / lob mode
 *       at the RPM calculated from distance to the shuttle target
 *       ({@link SK26DrumLauncher#runAtDistanceCommand}).</li>
 * </ul>
 *
 * <p>When neither button is held the subsystem's default command
 * ({@code idleCommand}) takes over automatically.
 */
public class SK26DrumLauncherBinder implements CommandBinder {

    private final Optional<SK26DrumLauncher> m_launcherContainer;
    private final Optional<SKVision> m_visionContainer;

    private final Trigger scoreTrigger = kRTrigger.button;
    private final Trigger shuttleTrigger = kRBbutton.button;

    /**
     * @param m_launcherContainer optional drum launcher subsystem
     * @param m_visionContainer   optional vision subsystem (needed for
     *                            distance-based scoring)
     */
    public SK26DrumLauncherBinder(
            Optional<SK26DrumLauncher> m_launcherContainer,
            Optional<SKVision> m_visionContainer) {
        this.m_launcherContainer = m_launcherContainer;
        this.m_visionContainer = m_visionContainer;
    }

    @Override
    public void bindButtons() {
        if (!m_launcherContainer.isPresent() || !m_visionContainer.isPresent()) {
            return;
        }

        SK26DrumLauncher launcher = m_launcherContainer.get();
        SKVision vision = m_visionContainer.get();

        // Right Trigger held → score mode (RPM from vision distance)
        scoreTrigger.whileTrue(
            launcher.runAtDistanceCommand(() -> vision.getDistanceToHub(), SK26DrumLauncher.ShotMode.SCORE)
                .withName("LauncherScoreAtDistance"));

        // Right Bumper held → shuttle / lob mode (RPM from distance to shuttle target)
        shuttleTrigger.whileTrue(
            launcher.runAtDistanceCommand(() -> vision.getDistanceTo(DrumLauncherConstants.kShuttleTarget), SK26DrumLauncher.ShotMode.SHUTTLE)
                .withName("LauncherShuttle"));
    }
}
