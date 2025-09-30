package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RunSalvageIntakeCommand;
import frc.robot.subsystems.SKSalvageIntake;
import static frc.robot.Ports.OperatorPorts.kSalvageIntakeTrigger;

public class SKSalvageIntakeBinder implements CommandBinder{
    Optional<SKSalvageIntake> m_intakeContainer;
    SKSalvageIntake m_intake;

    Trigger intakeTrigger;

    public SKSalvageIntakeBinder(Optional<SKSalvageIntake> m_intakeContainer) {
        this.m_intakeContainer = m_intakeContainer;

        intakeTrigger = kSalvageIntakeTrigger.button;
    }

    @Override
    public void bindButtons() {
        if(m_intakeContainer.isEmpty()) {
            return;
        }
        m_intake = m_intakeContainer.get();

        intakeTrigger.whileTrue(new RunSalvageIntakeCommand(m_intake));
    }
    
}
