// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  TalonSRX left1 = new TalonSRX(1);
  TalonSRX left2 = new TalonSRX(2);
  TalonFX left3 = new TalonFX(3);

  TalonSRX right1 = new TalonSRX(4);
  TalonSRX right2 = new TalonSRX(5);
  TalonFX right3 = new TalonFX(6);
  //  TalonSRX mytalon2 = new TalonSRX(1);
  //  TalonSRX mytalon3 = new TalonSRX(2);
  //  TalonSRX mytalon4 = new TalonSRX(3);
  Joystick _joystick = new Joystick(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double stick = -_joystick.getRawAxis(1);
    double otherStick = _joystick.getRawAxis(3);
    left1.set(ControlMode.PercentOutput, stick);
    left2.set(ControlMode.PercentOutput, stick);
    left3.set(ControlMode.PercentOutput, stick);
  //  mytalon2.set(ControlMode.PercentOutput, stick);
  
    right1.set(ControlMode.PercentOutput, otherStick);
    right2.set(ControlMode.PercentOutput, otherStick);
    right3.set(ControlMode.PercentOutput, otherStick);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
