package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Konstants.TurretConstants.turretConfig;
import static frc.robot.Ports.TurretPorts.kTurretMotorPort;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Field;

public class SP25Turret extends SubsystemBase {
    private SparkMax turretMotor = new SparkMax(kTurretMotorPort.ID, MotorType.kBrushless);
    private SparkClosedLoopController pidController = turretMotor.getClosedLoopController();
    private RelativeEncoder encoder = turretMotor.getEncoder();
    private Rotation2d targetRotation = Rotation2d.kZero;
    private Supplier<Rotation2d> pigeon = () -> (SKMecanumDrive.m_pigeon.getRotation2d());
    private Supplier<Double> turretAngle = () -> (Rotations.of(encoder.getPosition()).in(Degrees));

    public SP25Turret() {
        turretMotor.configure(turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        
        encoder.setPosition(0);

        this.setDefaultCommand(new InstantCommand(() -> resetFieldRelativeAngle(), this));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Target", targetRotation.getDegrees());
    }

    public void resetEncoder() {
        if (Field.isRed()) {
            encoder.setPosition(Degrees.of(180).in(Rotations));
        }
        else {
            encoder.setPosition(Degrees.of(0).in(Rotations));
        }
    }

    public void setEncoderAngle(double degrees) {
        encoder.setPosition(Degrees.of(degrees).in(Rotations));
    }
    public void setAngleRotations(double rotations) {
        pidController.setReference(rotations, ControlType.kMAXMotionPositionControl);
    }
    
    public void setAngleDegrees(double targetAngle) {
        // Get the turret's current angle in degrees
        double currentAngle = turretAngle.get();

        // "If the sign of the current angle and target angle aren't opposite"
        if(Math.signum(currentAngle) + Math.signum(targetAngle) != 0) {
            // Run the method as usual
            setAngleRotations(Degrees.of(targetAngle).in(Rotations));
        }
        else {
            // Check to see how far beyond 180 degrees the turret plans to rotate
            double targetAngle180DegDiff = 180 - Math.abs(targetAngle);

            // If it is attempting to rotate more than 35 degrees beyond 180 degrees, use the default method to rotate the turret
            if(targetAngle180DegDiff > 35) {
                // This effectively snaps the turret around the long way, avoiding "snapping its own neck"
                setAngleRotations(Degrees.of(targetAngle).in(Rotations));
                return;
            }

            double newTargetAngle;
            // Create a new, temporary target angle that is cocentric with the original target angle
            if(Math.abs(currentAngle) < 180) {
                // Find the degrees difference in order for the turret to cross beyond 180 
                // degrees and also reach the target angle 
                double totalAngleDifference = (180 - Math.abs(currentAngle)) + targetAngle180DegDiff;

                newTargetAngle = currentAngle + (totalAngleDifference * Math.signum(currentAngle));
            }
            // If the current angle has already passed beyond 180 degrees
            else {
                newTargetAngle = Math.signum(currentAngle) * (180 + Math.abs(targetAngle180DegDiff));
            }
            setAngleRotations(Degrees.of(newTargetAngle).in(Rotations));
        }

    }

    public void setFieldRelativeAngle(Rotation2d rotation) {
        targetRotation = rotation;

        setAngleDegrees(pigeon.get().unaryMinus().plus(targetRotation).getDegrees());
    }

    public void resetFieldRelativeAngle() {
        if(Field.isRed()) {
            setFieldRelativeAngle(Rotation2d.kCCW_90deg);
        }
        else {
            setFieldRelativeAngle(Rotation2d.kCW_90deg);
        }
    }
}
