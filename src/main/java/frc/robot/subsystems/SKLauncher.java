package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Ports.LauncherPorts.kLauncherMotorPort;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SKLauncher extends SubsystemBase{

    SparkMax motor = new SparkMax(kLauncherMotorPort.ID, MotorType.kBrushless);
    SparkBaseConfig motorConfig;

    public SKLauncher() {
        motorConfig.inverted(false).idleMode(IdleMode.kCoast).smartCurrentLimit(60);
        motorConfig.openLoopRampRate(0.1);

        motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Runs the intake motor at whatever speed is passed into the method
     * @param speed The speed to run the motor at
     */
    public void runLauncher(double speed) {
        motor.set(speed);
    }

    /**
     * Stops the intake motor
     */
    public void stopLauncher() {
        motor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LauncherSpeedRPM", motor.getEncoder().getVelocity());
    }

}