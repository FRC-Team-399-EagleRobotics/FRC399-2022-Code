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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.math.controller.BangBangController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

public class ShooterSubsystem extends SubsystemBase {
  private Solenoid hoodSolenoid;
  private TalonFX shooterL, shooterR;

<<<<<<< HEAD
    /* Joysticks */
    Joystick controller = new Joystick(0); //Controller
    // Joysticks
    Joystick _JoystickL = new Joystick(2);
    Joystick _JoystickR = new Joystick(1);

  //Solenoid
  private final Solenoid hoodSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 6);

  //Motor Setup
  WPI_TalonFX ShooterL = new WPI_TalonFX(13);
  WPI_TalonFX ShooterR = new WPI_TalonFX(14);
  MotorControllerGroup shooterMotors = new MotorControllerGroup(ShooterL, ShooterR);
=======
  //Motor Setup -- Keep this hopefully doesn't bother anybody
  BangBangController BANGshooter = new BangBangController();
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(10, 20);
>>>>>>> 10f7ef45a0d5a183f407e88ce5e65add89222c94

  // Variables to store shooter velocity and position
  double vel = 0.0;
  boolean pos = false;

  /**
   * Constructor
   */
  public ShooterSubsystem()
  {
    //Solenoid
    Solenoid hoodSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Shooter.hoodSolenoid_ID);

    // Motors
    shooterL = new TalonFX(Constants.Shooter.shooterL_ID);
    shooterR = new TalonFX(Constants.Shooter.shooterL_ID);
  }

  public void nearShot() {
    setVel(8500);
    setHood(false);
  }

  public void midShot() {
    setVel(4000);
    setHood(true);
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
    shooterL.set(ControlMode.Velocity, v);
    shooterR.set(ControlMode.Velocity, v);
  }

  public void setHood(boolean p)
  {
    pos = p;
    hoodSolenoid.set(p);
  }

  

}
