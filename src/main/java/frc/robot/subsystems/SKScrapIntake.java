package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Ports.IntakePorts.kIntakeMotorPort;

public class SKScrapIntake extends SubsystemBase{

    SparkMax motor;
    SparkBaseConfig motorConfig;

    public SKScrapIntake() {
        motor = new SparkMax(kIntakeMotorPort.ID, MotorType.kBrushless);
        
        motorConfig.inverted(false).idleMode(IdleMode.kCoast);

        motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Runs the intake motor at whatever speed is passed into the method
     * @param speed The speed to run the motor at
     */
    public void runIntake(double speed) {
        
    }

    /**
     * Stops the intake motor
     */
    public void stopIntake() {

    }

    @Override
    public void periodic() {

    }

}
