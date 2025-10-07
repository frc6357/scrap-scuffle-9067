package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SKLauncher;

public class RunLauncherCommandContinuous extends Command {
    private SKLauncher m_launcher;
    private double speed;

    public RunLauncherCommandContinuous(SKLauncher m_launcher, double speed) {
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
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}