package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RunLauncherCommand;
import frc.robot.subsystems.SKLauncher;
import static frc.robot.Ports.OperatorPorts.kLauncherTrigger;

public class SKLauncherBinder implements CommandBinder{
    Optional<SKLauncher> m_launcherContainer;
    SKLauncher m_launcher;

    Trigger launcherTrigger;

    public SKLauncherBinder(Optional<SKLauncher> m_launcherContainer) {
        this.m_launcherContainer = m_launcherContainer;

        launcherTrigger = kLauncherTrigger.button;
    }

    @Override
    public void bindButtons() {
        if(m_launcherContainer.isEmpty()) {
            return;
        }
        m_launcher = m_launcherContainer.get();

        launcherTrigger.whileTrue(new RunLauncherCommand(m_launcher, 0.7));
    }
    
}