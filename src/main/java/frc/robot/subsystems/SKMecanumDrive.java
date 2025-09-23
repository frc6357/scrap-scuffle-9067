package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Konstants.MecanumDriveConstants.kFrontLeftLocation;
import static frc.robot.Konstants.MecanumDriveConstants.kFrontRightLocation;
import static frc.robot.Konstants.MecanumDriveConstants.kMaxSpeed;
import static frc.robot.Konstants.MecanumDriveConstants.kWheelRadius;
import static frc.robot.Konstants.MecanumDriveConstants.kBackLeftLocation;
import static frc.robot.Konstants.MecanumDriveConstants.kBackRightLocation;
import static frc.robot.Ports.DrivePorts.kPigeonPort;

import java.util.function.DoubleConsumer;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;


public class SKMecanumDrive extends SubsystemBase {
    Pigeon2 m_pigeon;
    // Creating kinematics object using the wheel locations.
    MecanumDriveKinematics m_kinematics;
    MecanumDriveOdometry m_odometry;

    DoubleConsumer frontLeft = speed -> setFrontLeft(speed);
    DoubleConsumer frontRight = speed -> setFrontRight(speed);
    DoubleConsumer backLeft = speed -> setBackLeft(speed);
    DoubleConsumer backRight = speed -> setBackRight(speed);
    SparkMax frontLeftMotor;
    SparkMax frontRightMotor;
    SparkMax backLeftMotor;
    SparkMax backRightMotor;
    SparkMaxConfig fLConfig;
    SparkMaxConfig fRConfig;
    SparkMaxConfig bLConfig;
    SparkMaxConfig bRConfig;


    public SKMecanumDrive(
        SparkMax frontLeftMotor, SparkMax frontRightMotor,
        SparkMax backLeftMotor, SparkMax backRightMotor
        ) 
        {
        this.frontLeftMotor = frontLeftMotor; this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor; this.backRightMotor = backRightMotor;

        fLConfig.idleMode(IdleMode.kCoast).inverted(false).openLoopRampRate(0.4).smartCurrentLimit(40, 50);
        fRConfig.idleMode(IdleMode.kCoast).inverted(false).openLoopRampRate(0.4).smartCurrentLimit(40, 50);
        bLConfig.idleMode(IdleMode.kCoast).inverted(false).openLoopRampRate(0.4).smartCurrentLimit(40, 50);
        bRConfig.idleMode(IdleMode.kCoast).inverted(false).openLoopRampRate(0.4).smartCurrentLimit(40, 50);

        this.frontLeftMotor.configure(fLConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        this.frontRightMotor.configure(fRConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        this.backLeftMotor.configure(bLConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        this.backRightMotor.configure(bRConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        m_pigeon = new Pigeon2(kPigeonPort.ID, kPigeonPort.bus);
        m_pigeon.reset();

        m_kinematics = new MecanumDriveKinematics(
            kFrontLeftLocation, kFrontRightLocation,
            kBackLeftLocation, kBackRightLocation
        );

        m_odometry = new MecanumDriveOdometry(
            m_kinematics,
            getIMURotation(),
            getWheelPositions(),
            new Pose2d(0.0, 0.0, new Rotation2d())
        );
    }

    @Override
    public void periodic() {
        m_odometry.update(getIMURotation(), getWheelPositions());
    }

    /**
     * Sets the <b>robot-relative</b> velocity for the drivetrain to target.
     * @param vx Translational X velocity
     * @param vy Translation Y velocity
     * @param omega Rotational velocity (rad/sec)
     * @return
     */
    public void setControl(double vx, double vy, double omega) {
        MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(
            new ChassisSpeeds(vx, vy, omega)
        );
        //TODO: Add a desaturation once a max motor velocity has been determined
        
        frontLeft.accept(wheelSpeeds.frontLeftMetersPerSecond);
        frontRight.accept(wheelSpeeds.frontRightMetersPerSecond);
        backLeft.accept(wheelSpeeds.rearLeftMetersPerSecond);
        backRight.accept(wheelSpeeds.rearRightMetersPerSecond);
    }

    /**
     * Sets the <b>robot-relative</b> velocity for the drivetrain to target.
     * @param speeds The ChassisSpeeds object to handle
     */
    public void setControl(ChassisSpeeds speeds) {
        MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
        //TODO: Add a desaturation once a max motor velocity has been determined
        
        frontLeft.accept(wheelSpeeds.frontLeftMetersPerSecond);
        frontRight.accept(wheelSpeeds.frontRightMetersPerSecond);
        backLeft.accept(wheelSpeeds.rearLeftMetersPerSecond);
        backRight.accept(wheelSpeeds.rearRightMetersPerSecond);
    }

    // Uses the pigeon IMU to return the robot's rotation
    public Rotation2d getIMURotation() {
        return m_pigeon.getRotation2d();
    }

    public MecanumDriveWheelPositions getWheelPositions() {
        return new MecanumDriveWheelPositions(
            getDistance(frontLeftMotor.getEncoder()), getDistance(frontRightMotor.getEncoder()),
            getDistance(backLeftMotor.getEncoder()), getDistance(backRightMotor.getEncoder())
        );
    }

    public void resetIMU() {
        m_pigeon.reset();
    }

    /**
     * Resets both the IMU and odometry rotation
     */
    public void resetRotation() {
        m_pigeon.reset();
        m_odometry.resetRotation(getIMURotation());
    }

    public void resetPose(Pose2d pose) {
        m_odometry.resetPose(pose);
    }

    /**
     * Returns the meters travelled from a wheel motor by multiplying the number of rotations
     * by 2Pi to convert to radians, then multiplying the radians by the radius of the wheel in meters
     * @param encoder The encoder to read
     * @return The meters travelled by the wheel's encoder
     */
    private double getDistance(RelativeEncoder encoder) {
        return (encoder.getPosition() * 2 * Math.PI) * kWheelRadius;
    }

    private void setFrontLeft(double speed) {
        frontLeftMotor.set(speed/kMaxSpeed); // Divides by maximum linear speed to find the percentage to pass into the set() method
    }
    private void setFrontRight(double speed) {
        
    }
    private void setBackLeft(double speed) {
        
    }
    private void setBackRight(double speed) {

    }
}
