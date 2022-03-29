// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonomousClimber;
import frc.robot.commands.AutonomousConveyor;
import frc.robot.commands.AutonomousDrive;
import frc.robot.commands.AutonomousIntake;
import frc.robot.commands.AutonomousShooter;
import frc.robot.commands.Climber;
import frc.robot.commands.ConveyorCmd;
import frc.robot.commands.ExtendIntake;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.Tankdrive;
import frc.robot.commands.WaitSecondsCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...\

  public static Joystick leftJoy = new Joystick(1);
  public static Joystick rightJoy = new Joystick(2);
  public static Joystick operator = new Joystick(0);
  public double stickL = RobotContainer.leftJoy.getRawAxis(1);
  public double stickR = RobotContainer.rightJoy.getRawAxis(1); 

  //-----Intake------
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  // Command
  private final ExtendIntake m_ExtendIntake = new ExtendIntake(m_intakeSubsystem, 0, false);
  private final AutonomousIntake m_autoIntake = new AutonomousIntake(m_intakeSubsystem, 0, false, 0, false);

  //-----Conveyor----
  private final ConveyorSubsystem m_conveyorSubsystem = new ConveyorSubsystem();
  //Commands
  private final ConveyorCmd m_conveyorCmd = new ConveyorCmd(m_conveyorSubsystem, 0, 0);
  private final AutonomousConveyor m_AutonomousConveyor = new AutonomousConveyor(m_conveyorSubsystem, 0, 0, 0);

  //-----Shooter-----
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  // Commands
  private final ShooterCmd m_shooterCmd = new ShooterCmd(m_shooterSubsystem, 1, false);
  private final AutonomousShooter m_AutonomousShooter = new AutonomousShooter(m_shooterSubsystem, 0, false, 0);

  //----Drivetrain-----
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final Tankdrive m_tankdrive = new Tankdrive(m_drivetrainSubsystem, 0, 0);
  private final AutonomousDrive m_autodrive = new AutonomousDrive(m_drivetrainSubsystem, 0, 0, 0);

  //-----Climber------ 
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  //Commands
  private final Climber m_climberCmd = new Climber(m_climberSubsystem, 0);
  private final AutonomousClimber m_AutonomousClimber = new AutonomousClimber(m_climberSubsystem, 0, false, 0);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_intakeSubsystem.setDefaultCommand(m_ExtendIntake);
    m_conveyorSubsystem.setDefaultCommand(m_conveyorCmd);
    m_shooterSubsystem.setDefaultCommand(m_shooterCmd);
    m_climberSubsystem.setDefaultCommand(m_climberCmd);
    m_drivetrainSubsystem.setDefaultCommand(m_tankdrive);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public SequentialCommandGroup getAutonomousCommand() {

    // CommandGroup auton = new CommandGroup();
    
    // Drive code
    AutonomousDrive autoDrive = new AutonomousDrive(m_drivetrainSubsystem, -0.25, -.25, 4.0); // Reverse for .25 for 4 secs
    WaitSecondsCommand wait = new WaitSecondsCommand(m_drivetrainSubsystem, 10.0); // Wait for 10 secs
    AutonomousIntake autoIntake = new AutonomousIntake(m_intakeSubsystem, 1, true,  4, true); // Intake TODO Maybe don't use timer? 
    ParallelCommandGroup grabBall = new ParallelCommandGroup(autoDrive, autoIntake); // Bundles commands to run at the same time
    SequentialCommandGroup auton = new SequentialCommandGroup(grabBall, wait); // Bundles the commands

    return auton; // Runs the commands
    
  }
}
