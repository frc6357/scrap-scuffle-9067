package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Ports.DrivePorts.kPigeonPort;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;

public final class Konstants
{

    // public static final class PracticeSwerveConstants
    // {
    //     //8 swerve motor IDs for Ports
    //     public static final int kFrontLeftDriveMotorId = 1;
    //     public static final int kFrontLeftTurnMotorId = 2;
    //     public static final int kFrontRightDriveMotorId = 3;
    //     public static final int kFrontRightTurnMotorId = 4;
    //     public static final int kBackLeftDriveMotorId = 5;
    //     public static final int kBackLeftTurnMotorId = 6;
    //     public static final int kBackRightDriveMotorId = 7;
    //     public static final int kBackRightTurnMotorId = 8;

    //     //swerve chassis width and length in inches
    //     public static final int kChassisLength = 28;
    //     public static final int kChassisWidth = 28;

    //     //PID Constants
    //     public static final double kDriveP = 0.1;
    //     public static final double kDriveI = 0;
    //     public static final double kDriveD = 0;
    //     public static final double kTurnP = 0.1;
    //     public static final double kTurnI = 0;
    //     public static final double kTurnD = 0;
    // }

    public static final class MecanumDriveConstants {
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(3.78);

        public static final double kMaxSpeed = kSpeedAt12Volts.in(MetersPerSecond);
        public static final double kMaxAngularSpeed = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        public static final double kMaxAngularSpeedDegrees = RadiansPerSecond.of(kMaxAngularSpeed).in(DegreesPerSecond);
        public static final double kWheelRadius = 0.0762; // in meters
        public static final double kMaxWheelSpeed = 4.5; // m/s

        public static final int kPigeonID = kPigeonPort.ID;

        public static final ClosedLoopConfig driveMotorPIDConfig = new ClosedLoopConfig().feedbackSensor(FeedbackSensor.kPrimaryEncoder).p(0.1).i(0).d(0).velocityFF(0.124).outputRange(-1, 1);

        // Locations of the wheels relative to the robot center. (In meters)
        public static final Translation2d kFrontLeftLocation = new Translation2d(0.381, 0.381);
        public static final Translation2d kFrontRightLocation = new Translation2d(0.381, -0.381);
        public static final Translation2d kBackRightLocation = new Translation2d(-0.381, -0.381);
        public static final Translation2d kBackLeftLocation = new Translation2d(-0.381, 0.381);
    }

    public static final class AutoConstants
    {
        // PID Constants
        public static final PIDConstants kTranslationPIDConstants = new PIDConstants(6, 0, 0);
        public static final PIDConstants kRotationPIDConstants    = new PIDConstants(6, 0.4, 0);

        public static final PPHolonomicDriveController pathConfig = new PPHolonomicDriveController(kTranslationPIDConstants, kRotationPIDConstants);
    }

    public static final class IntakeConstants {
        
    }

    public static final class IOConstants {
        public static final double kJoystickDeadband = 0.15;
        public static final double kSlowModePercent  = 0.3;
        public static final double kSlowModeRotationPercent = 0.5;
    }

    public static final class LightConstants
    {
        public static final int numLedOnBot = 240;
        public static final double kLightsOffBrightness = 0.0;
        public static final double kLightsOnBrightness = 0.5;
    }

    public static final class ExampleConstants
    {
        public static final double kExampleSpeed = 0.5;
    }
    
    

    /** The file that is used for system instantiation at runtime */
    public static final String SUBSYSTEMFILE = "Subsystems.json";
}

