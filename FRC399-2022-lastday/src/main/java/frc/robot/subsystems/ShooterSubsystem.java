// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.BangBangController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

public class ShooterSubsystem extends SubsystemBase {
  private Solenoid hoodSolenoid;
  private TalonFX shooterL, shooterR;
  private Timer m_timer;

  //Motor Setup -- Keep this hopefully doesn't bother anybody
  //BangBangController BANGshooter = new BangBangController();
  //SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(10, 20);

  // Variables to store shooter velocity and position
  double vel = 0.0;
  boolean pos = false;

  /**
   * Constructor
   */
  public ShooterSubsystem()
  {
    //Solenoid
    hoodSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Shooter.hoodSolenoid_ID);

    // Motors
    shooterL = new TalonFX(Constants.Shooter.shooterL_ID);
    shooterR = new TalonFX(Constants.Shooter.shooterR_ID);

    
  }

  public void lowShot() {
    setVel(1);
    setHood(false);
  }

  public void highShot() {
    setVel(1);
    setHood(true);
  }

  public void endShooter() {
    setVel(0);
    setHood(false);
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
  }

  // Basically Pwr for the shooters
  public void setVel(double v)
  {
    vel = v;
    shooterL.set(ControlMode.PercentOutput, v);
    shooterR.set(ControlMode.PercentOutput, -v);
  }

  public void setHood(boolean p)
  {
    pos = p;
    hoodSolenoid.set(p);
  }
  
  public void autoFire(double v, boolean p, double t)  {
    m_timer.reset();
    m_timer.start();
    if (m_timer.get() < t) {
      setVel(v);
    } else {
      setVel(v);
    }
  }
  
  

}
