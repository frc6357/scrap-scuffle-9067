package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SKSalvageIntake;

public class RunSalvageIntakeCommandContinuous extends Command {
    private SKSalvageIntake m_intake;

    public RunSalvageIntakeCommandContinuous(SKSalvageIntake m_intake) {
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
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
