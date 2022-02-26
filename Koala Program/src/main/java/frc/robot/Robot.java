// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// This version was arranged by Cetis Casillas 
// Shooter/Solenoid: Darian Racster
// Bit on the Climber: Carlo Rafeh

// TODO Nvm the climber is user controlled
// TODO Going to use two motors on the shooter but just for more power
// TODO Fix the motor control group - Cetis/Jeremy knows

package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Timer;

//There might be some unnessary imports
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

// Compressor/Solenoid
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  // Controller used 2
  // Joystick used 1 and 2


  /* Joysticks */
  Joystick controller = new Joystick(0); //Controller
  // Joysticks
  Joystick _JoystickL = new Joystick(2);
  Joystick _JoystickR = new Joystick(1);


  //Climber motor
  WPI_TalonSRX ClimberL = new WPI_TalonSRX(9);
  WPI_TalonSRX ClimberR = new WPI_TalonSRX(10);

  /* Left Motors */
  WPI_TalonSRX _talonLB = new WPI_TalonSRX(1);
  WPI_TalonSRX _talonLF = new WPI_TalonSRX(2);
  WPI_TalonFX _talonTL = new WPI_TalonFX(3); 

  /* Right Motors */
  WPI_TalonSRX _talonRB = new WPI_TalonSRX(4);
  WPI_TalonSRX _talonRF = new WPI_TalonSRX(5);
  WPI_TalonFX _talonTR = new WPI_TalonFX(6);

  //Shooter motor
  WPI_TalonFX ShooterL = new WPI_TalonFX(13);
  WPI_TalonFX ShooterR = new WPI_TalonFX(14);

  // Intake motor 
  WPI_TalonSRX IntakeMotor = new WPI_TalonSRX(7);

  // Conveyor motors
  WPI_TalonSRX BottomConveyorMotor = new WPI_TalonSRX(15);
  WPI_TalonSRX TopConveyorMotor = new WPI_TalonSRX(16);

  //Bundling Climbers - This was meant to save one line of code but may not work or implemented right in autonomousPeriodic for climbers
  MotorControllerGroup Climbers = new MotorControllerGroup(ClimberL, ClimberR);

  // Timer & Encoder
  Timer m_timer = new Timer();

  // Bundling the motors
  MotorControllerGroup leftMotors = new MotorControllerGroup(_talonLB, _talonLF, _talonTL);
  MotorControllerGroup rightMotors = new MotorControllerGroup(_talonRB, _talonRF, _talonTR);
  MotorControllerGroup shooterMotors = new MotorControllerGroup(ShooterL, ShooterR);

  DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

  boolean toggleOn = false;
  boolean togglePressed = false;
  boolean toggleOn2 = false;
  boolean togglePressed2 = false;
  boolean toggleOn3 = false;
  boolean togglePressed3 = false;

// Solenoid corresponds to a single solenoid.
Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
private final Solenoid intakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 7);
private final Solenoid shooterSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
private final Solenoid m_solenoid3 = new Solenoid(PneumaticsModuleType.CTREPCM, 2);


/*private static final int kSolenoidButton = 4;
private static final int kDoubleSolenoidForward = 2;
private static final int kDoubleSolenoidReverse = 3;*/

// created toggle switch to activate solenoid
/********INTAKE*****************/ 
public void updateToggle()
{
if(controller.getRawButton(2)) {
  if(!togglePressed){
    // Activating solenoid
    toggleOn = !toggleOn;
    Timer.delay(0.5);
    togglePressed = true;
  } else {
    togglePressed = false;
  }

 }
}
public void updateToggle2()
{
  if(controller.getRawButton(3)) {
    if(!togglePressed2){
      // Activating solenoid
      toggleOn2 = !toggleOn2;
      Timer.delay(0.5);
      togglePressed2 = true;
    } else {
      togglePressed2 = false;
    }

 }
}
  @Override
  public void robotInit() {
  }

  @Override
  public void teleopInit() {
    pcmCompressor.enableDigital();
  }

  @Override
   public void autonomousInit() {
      m_timer.reset();
      m_timer.start();
    }

  /** This function is called periodically during autonomous. */

  @Override
  public void autonomousPeriodic() {
      
  }
  @Override
  public void teleopPeriodic() {
    /********INTAKE*********/
    //pnuematics 
    updateToggle();
    // toggle switch for solenoid valve
    if(toggleOn){
      // Might have to use double solenoid but theres problems using true or false.
      intakeSolenoid.set(true);
    }else{
      intakeSolenoid.set(false);
    }

    updateToggle2();
    // toggle switch for solenoid valve
    if(toggleOn2){
      // Might have to use double solenoid but theres problems using true or false.
      shooterSolenoid.set(true);
    }else{
      shooterSolenoid.set(false);
    }

    /*
     * In order to set the double solenoid, if just one button
     * is pressed, set the solenoid to correspond to that button.
     * If both are pressed, set the solenoid will be set to Forwards.
     */


    /********DRIVE BASE********/
    /* Drive Control */
    // Sticks
    double stickL = _JoystickL.getRawAxis(1);
    double stickR = _JoystickR.getRawAxis(1);

    // Gamepad
    double joystickL = _JoystickL.getRawAxis(1);
    double joyR = _JoystickR.getRawAxis(1);

    /* Tank Drive */ 
    drive.tankDrive((stickL*-1), stickR);

    /* Arcade drive */
    //drive.arcadeDrive(controller.getX(), controller.getY());

    /************SHOOTER**********/
    // TODO We're probably going to use pnuematics to angle the shoot
    // Shooter variable
    double shooterVelocity = ShooterL.getSelectedSensorVelocity();

    // Shooter and Conveyor code
    int MaxVelocity = 8500;
    double scalar = 1.0;

    if(togglePressed2)
    {
      scalar = .4;
      MaxVelocity = 4000;
    }else
    {
      scalar = 1;
      MaxVelocity =8500;
    }
    System.out.println(shooterVelocity);
    if(controller.getRawButton(8)) {
      if (shooterVelocity >= MaxVelocity) {
        ShooterL.set(.75 * scalar);
        ShooterR.set(-.75 * scalar);
        TopConveyorMotor.set(-0.5);
        BottomConveyorMotor.set(-0.5);
      }else{
        ShooterL.set(1 * scalar);
        ShooterR.set(-1* scalar);
      }
    }else if (controller.getRawButton(1)){
      BottomConveyorMotor.set(-.5);
      TopConveyorMotor.set(0.5);
    }else if (controller.getRawButton(4)){
      BottomConveyorMotor.set(0.5);
      TopConveyorMotor.set(0.5);
    }else{
      BottomConveyorMotor.set(0);
      TopConveyorMotor.set(0);
      ShooterL.set(0);
      ShooterR.set(0);
    }
    

    //Intake Motors
    if(controller.getRawButton(7))
    {
      IntakeMotor.set(-1);
    }else if(controller.getRawButton(5))
    {
      IntakeMotor.set(.5);
    }else
    {
      IntakeMotor.set(0);
    }



  }
}

