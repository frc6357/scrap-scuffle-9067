package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RunScrapIntakeCommand;
import frc.robot.subsystems.SKScrapIntake;
import static frc.robot.Ports.OperatorPorts.kIntakeTrigger;

public class SKScrapIntakeBinder implements CommandBinder{
    Optional<SKScrapIntake> m_intakeContainer;
    SKScrapIntake m_intake;

    Trigger intakeTrigger;

    public SKScrapIntakeBinder(Optional<SKScrapIntake> m_intakeContainer) {
        this.m_intakeContainer = m_intakeContainer;

        intakeTrigger = kIntakeTrigger.button;
    }

    @Override
    public void bindButtons() {
        if(m_intakeContainer.isEmpty()) {
            return;
        }
        m_intake = m_intakeContainer.get();

        intakeTrigger.whileTrue(new RunScrapIntakeCommand(m_intake));
    }
    
}
