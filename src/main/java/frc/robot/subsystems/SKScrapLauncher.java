package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static frc.robot.Ports.LauncherPorts.kLauncherMotorPort;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SKScrapLauncher extends SubsystemBase{

    private SparkMax motor;
    private SparkBaseConfig config;
    private double targetLauncherSpeed;

    public SKScrapLauncher() {
        motor = new SparkMax(kLauncherMotorPort.ID, MotorType.kBrushless);

        config.inverted(false).idleMode(IdleMode.kCoast);

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Runs the launcher at the designated speed
     * @param speed The speed to run the motor at
     */
    public void runLauncher(double speed) {
        targetLauncherSpeed = speed;
        motor.set(speed);
    }

    /**
     * Stops the launcher motor
     */
    public void stopLauncher() {
        motor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Target Launcher Speed", targetLauncherSpeed);
    }
}
