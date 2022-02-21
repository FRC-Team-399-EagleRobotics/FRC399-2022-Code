// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  
  // Controller used 2
  // Joystick used 1 and 2
  /* Joysticks */
  //Actually a controller
  Joystick _joystickL = new Joystick(2);

  /* Left Motors */
  WPI_TalonSRX _talonLB = new WPI_TalonSRX(4);
  WPI_TalonFX _talonLF = new WPI_TalonFX(6);

  /* Right Motors */
  WPI_TalonSRX _talonRB = new WPI_TalonSRX(1);
  WPI_TalonFX _talonRF = new WPI_TalonFX(5);
  Timer m_timer = new Timer();

  //Shooter motor
  WPI_TalonSRX Shooter = new WPI_TalonSRX(7);
 
  //Other junk
  Encoder encoder = new Encoder(0,1);
  

  // Bundling the motors
  MotorControllerGroup leftMotors = new MotorControllerGroup(_talonLB, _talonLF);
  MotorControllerGroup rightMotors = new MotorControllerGroup(_talonRB, _talonRF);

  DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    encoder.setDistancePerPulse(1./256.);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    /*
    if (encoder.getDistance() < 5) {
      drive.tankDrive(0.2*-1, 0.2);
    } else {
      drive.tankDrive(0, 0);
    }
    */
    // Basic drive forward autonomous 
    // Drive for 10 seconds
    if (m_timer.get() <= 10.0) {
      System.out.println("In while: " + m_timer.get());
      _talonRB.set(ControlMode.PercentOutput, 0.3);
    } else {
      _talonLF.DestroyObject();
      _talonLB.DestroyObject();
      _talonRB.DestroyObject();
      _talonRF.DestroyObject();
    }
    }

  public void shooterInit() {
    Shooter.set(0);
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    /* Left Motor */
    double stickL1 = _joystickL.getRawAxis(1);
    double stickR1 = _joystickL.getRawAxis(3);
    drive.tankDrive((stickL1*-1), stickR1);
    /*
    _talonLB.set(ControlMode.PercentOutput, (stickL1*-1));
    _talonLF.set(ControlMode.PercentOutput, (stickL1*-1));

    /* Right Motor 
    double stickR1 = _joystickL.getRawAxis(3);
    _talonRF.set(ControlMode.PercentOutput, (stickR1));
    _talonRB.set(ControlMode.PercentOutput, (stickR1));
    */

    // Shooter variable
    double shooterVelocity = Shooter.getSelectedSensorVelocity();

    // Tries to keep motor at same velocity while running
    if(_joystickL.getRawButton(8) == true) {
      if (shooterVelocity >= 20000.0) {
        Shooter.set(1900.0);
        System.out.println(shooterVelocity);
      }else{
        Shooter.set(20000.0);}
    }else{
      Shooter.set(0);
    }
    
  }
  
    
  

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
