package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Ports.IntakePorts.kIntakeMotorPort;

public class SKScrapIntake extends SubsystemBase{

    SparkMax motor = new SparkMax(kIntakeMotorPort.ID, MotorType.kBrushless);
    SparkBaseConfig motorConfig;

    public SKScrapIntake() {
        motorConfig.inverted(false).idleMode(IdleMode.kCoast).smartCurrentLimit(50);
        motorConfig.openLoopRampRate(0.2);

        motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Runs the intake motor at whatever speed is passed into the method
     * @param speed The s0peed to run the motor at
     */
    public void runIntake(double speed) {
        motor.set(speed);
    }

    /**
     * Stops the intake motor
     */
    public void stopIntake() {
        motor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ScrapIntakeSpeed", motor.getEncoder().getVelocity());
    }

}
