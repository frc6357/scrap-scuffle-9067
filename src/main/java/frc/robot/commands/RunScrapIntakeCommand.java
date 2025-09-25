package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SKScrapIntake;

public class RunScrapIntakeCommand extends Command {
    private SKScrapIntake m_intake;

    public RunScrapIntakeCommand(SKScrapIntake m_intake) {
        this.m_intake = m_intake;

        addRequirements(this.m_intake);
    }

    @Override
    public void initialize() {
        // Runs the intake at -100% speed
        m_intake.runIntake(-1);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        // Stops intake by targeting a velocity of 0%
        m_intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
