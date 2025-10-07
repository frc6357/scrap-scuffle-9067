package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SKLauncher;

public class RunLauncherCommand extends Command {
    private SKLauncher m_launcher;
    private double speed;

    public RunLauncherCommand(SKLauncher m_launcher, double speed) {
        this.m_launcher = m_launcher;

        addRequirements(this.m_launcher);
    }

    @Override
    public void initialize() {
        m_launcher.runLauncher(speed);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        // Stops intake by targeting a velocity of 0%
        m_launcher.stopLauncher();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}