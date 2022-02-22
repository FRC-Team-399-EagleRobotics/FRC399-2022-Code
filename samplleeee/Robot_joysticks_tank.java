// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
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
  Joystick _joystickL = new Joystick(1);
  Joystick _joystickR = new Joystick(0);

  /* Left Motors */
  TalonSRX _talonLB = new TalonSRX(4);
  TalonSRX _talonLF = new TalonSRX(3);

  /* Right Motors */
  TalonSRX _talonRB = new TalonSRX(1);
  TalonSRX _talonRF = new TalonSRX(2);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
  
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds

    }
  

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    /* Left Motor */
    double stickL1 = _joystickL.getRawAxis(1);
    _talonLB.set(ControlMode.PercentOutput, (stickL1*-1));
    _talonLF.set(ControlMode.PercentOutput, (stickL1*-1));

    /* Right Motor */
    double stickR1 = _joystickR.getRawAxis(1);
    _talonRF.set(ControlMode.PercentOutput, stickR1);
    _talonRB.set(ControlMode.PercentOutput, stickR1);
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
