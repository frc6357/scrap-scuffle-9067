// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.bindings.CommandBinder;
import frc.robot.bindings.SKSalvageIntakeBinder;
import frc.robot.bindings.SKScrapIntakeBinder;
import frc.robot.commands.RunSalvageIntakeCommand;
import frc.robot.commands.RunSalvageIntakeCommandContinuous;
import frc.robot.commands.RunScrapIntakeCommand;
import frc.robot.commands.RunScrapIntakeCommandContinuous;
import frc.robot.commands.StopSalvageIntakeCommand;
import frc.robot.commands.StopScrapIntakeCommand;
import frc.robot.subsystems.SKMecanumDrive;
import frc.robot.subsystems.SKSalvageIntake;
import frc.robot.subsystems.SKScrapIntake;
import frc.robot.bindings.SKLauncherBinder;
import frc.robot.bindings.SKMecanumBinder;
import frc.robot.commands.RunLauncherCommand;
import frc.robot.commands.RunLauncherCommandContinuous;
import frc.robot.commands.StopLauncherCommand;
import frc.robot.subsystems.SKLauncher;
import frc.robot.utils.SK25AutoBuilder;
import frc.robot.utils.SubsystemControls;
import frc.robot.utils.filters.FilteredJoystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Make your list of subsystem containers here
  // ex: public Optional<SKVision> m_visionContainer = Optional.empty();

  public Optional<SKMecanumDrive> m_driveContainer = Optional.empty();
  public Optional<SKScrapIntake> m_scrapIntakeContainer = Optional.empty();
  public Optional<SKSalvageIntake> m_salvageIntakeContainer = Optional.empty();
  public Optional<SKLauncher> m_launcherContainer = Optional.empty();

  // Then make static references to each subsystem you've added
  // ex: public static SKVision m_vision;
  public static SKLauncher m_launcher;
  public static SKMecanumDrive m_drive;
  public static SKScrapIntake m_scrapIntake;
  public static SKSalvageIntake m_salvageIntake;

  // The list containing all the command binding classes
  private List<CommandBinder> buttonBinders = new ArrayList<CommandBinder>();

  // An option box on shuffleboard to choose the auto path
  SendableChooser<Command> autoCommandSelector = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Creates all subsystems that are on the robot
    configureSubsystems();

    // sets up autos needed for pathplanner
    configurePathPlannerCommands();

    // Configure the trigger bindings
    configureButtonBindings();

    autoCommandSelector = SK25AutoBuilder.buildAutoChooser("P1Jolt");
    //set delete old files = true in build.gradle to prevent sotrage of unused orphans
    SmartDashboard.putData("Select an Auto", autoCommandSelector);
  }

  /**
     * Will create all the optional subsystems using the json file in the deploy directory
     */
    private void configureSubsystems()
    {
        File deployDirectory = Filesystem.getDeployDirectory();

        ObjectMapper mapper = new ObjectMapper();
        JsonFactory factory = new JsonFactory();

        try
        {
            // Looking for the Subsystems.json file in the deploy directory
            JsonParser parser =
                    factory.createParser(new File(deployDirectory, Konstants.SUBSYSTEMFILE));
                    SubsystemControls subsystems = mapper.readValue(parser, SubsystemControls.class);
                    
            // ex:
            // if(subsystems.isVisionPresent())
            // {
            //     m_visionContainer = Optional.of(new SKVision());
            //     m_vision = m_visionContainer.get();
            // }
            if(subsystems.isMecanumDrivePresent())
            {
                m_driveContainer = Optional.of(new SKMecanumDrive());
                m_drive = m_driveContainer.get();
            }
            if(subsystems.isScrapIntakePresent()) {
                m_scrapIntakeContainer = Optional.of(new SKScrapIntake());
                m_scrapIntake = m_scrapIntakeContainer.get();
            }

            if(subsystems.isSalvageIntakePresent()) {
                m_salvageIntakeContainer = Optional.of(new SKSalvageIntake());
                m_salvageIntake = m_salvageIntakeContainer.get();
            }


            if(subsystems.isLauncherPresent()) {
                m_launcherContainer = Optional.of(new SKLauncher());
                m_launcher = m_launcherContainer.get();
            }
        }
        catch (IOException e)
        {
            DriverStation.reportError("Failure to read Subsystem Control File!", e.getStackTrace());
        }
    }

  /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link FilteredJoystick}), and then
     * calling passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings()
    {

        // ex: buttonBinders.add(new SKVisionBinder(m_visionContainer))
        // Note: if your subsystem binder interacts/controls other subsystems, you can add its 
        //       respective container reference into the binder constructor.
        // ex: buttonBinders.add(new SKVisionBinder(m_visionContainer, m_driveContainer, m_launcherContainer))

        buttonBinders.add(new SKScrapIntakeBinder(m_scrapIntakeContainer));
        buttonBinders.add(new SKSalvageIntakeBinder(m_salvageIntakeContainer));
        buttonBinders.add(new SKLauncherBinder(m_launcherContainer));
        buttonBinders.add(new SKMecanumBinder(m_driveContainer));

        // Traversing through all the binding classes to actually bind the buttons
        for (CommandBinder subsystemGroup : buttonBinders)
        {
            subsystemGroup.bindButtons();
        }

    }

    private void configurePathPlannerCommands()
    {
        // Always check to see if the drivetrain is present for auto
        // It's kinda useless to create autonomous commands if there's no drivebase
        // to move the robot around the field...

        /* ex:
         * Nest the other subsystem checking if-statements inside the drivetrain if-statement
         * 
         * if(m_driveContainer.isPresent()) {
         *      if(m_launcherContainer.isPresent()) {
         *            // This line configures a launching command to be used in autonomous and feeds in
         *            // a medium motor speed value
         *            NamedCommands.registerCommand("RunLauncherMediumCommand", new RunLauncherCommand(kMediumSpeed));
         *      }
         * }
         */

        if(m_driveContainer.isPresent()) {            
            if(m_salvageIntakeContainer.isPresent()) {
                NamedCommands.registerCommand("RunSalvageIntakeCommand", new RunSalvageIntakeCommand(m_salvageIntake));
                NamedCommands.registerCommand("RunSalvageIntakeCommandContinuous", new RunSalvageIntakeCommandContinuous(m_salvageIntake));
                NamedCommands.registerCommand("StopSalvageIntakeCommand", new StopSalvageIntakeCommand(m_salvageIntake));
            }
            if(m_scrapIntakeContainer.isPresent()) {
                NamedCommands.registerCommand("RunScrapIntakeCommand", new RunScrapIntakeCommand(m_scrapIntake));
                NamedCommands.registerCommand("RunScrapIntakeCommandContinuous", new RunScrapIntakeCommandContinuous(m_scrapIntake));
                NamedCommands.registerCommand("StopScrapIntakeCommand", new StopScrapIntakeCommand(m_scrapIntake));
            }
            if(m_launcherContainer.isPresent()) {
               NamedCommands.registerCommand("RunLauncherCommand70Pct", new RunLauncherCommand(m_launcher, 0.7));
               NamedCommands.registerCommand("RunLauncherContinuous70Pct", new RunLauncherCommandContinuous(m_launcher, 0.7));
               NamedCommands.registerCommand("StopLauncherCommand", new StopLauncherCommand(m_launcher));
            }
        }

    }

  /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * <p>
     * This method loads the auto when it is called, however, it is recommended
     * to first load your paths/autos when code starts, then return the
     * pre-loaded auto/path.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return Commands.sequence(Commands.waitSeconds(0.01), autoCommandSelector.getSelected());
    }

    public void testPeriodic(){

    }
    public void testInit(){

    }

    public void matchInit()
    {
    
    }

    public void teleopInit()
    {
       
    }
    public void autonomousInit()
    {
     
    }
}
