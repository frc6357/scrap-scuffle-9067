package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Ports.IntakePorts.kIntakeMotorPort;

public class SKScrapIntake extends SubsystemBase{

    SparkMax motor;

    public SKScrapIntake() {


        motor = new SparkMax(kIntakeMotorPort.ID, MotorType.kBrushless);
    }

}
