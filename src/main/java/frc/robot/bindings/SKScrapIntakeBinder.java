package frc.robot.bindings;

import java.util.Optional;

import frc.robot.subsystems.SKScrapIntake;

public class SKScrapIntakeBinder implements CommandBinder{
    Optional<SKScrapIntake> m_intakeContainer;
    SKScrapIntake m_intake;

    public SKScrapIntakeBinder(Optional<SKScrapIntake> m_intakeContainer) {
        this.m_intakeContainer = m_intakeContainer;
    }

    @Override
    public void bindButtons() {
        if(m_intakeContainer.isEmpty()) {
            return;
        }

        m_intake = m_intakeContainer.get();
    }
    
}
