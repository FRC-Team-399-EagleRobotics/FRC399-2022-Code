// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.Timer;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Compressor;

public class ShooterSubsystem extends SubsystemBase {
  // TODO: instantiate shooter motor controller and hood solenoid here!

    /* Joysticks */
    Joystick controller = new Joystick(0); //Controller
    // Joysticks
    Joystick _JoystickL = new Joystick(2);
    Joystick _JoystickR = new Joystick(1);

  //Solenoid and Compressor Setup
  Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private final Solenoid shooterSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 6);

  //Motor Setup
  WPI_TalonFX ShooterL = new WPI_TalonFX(13);
  WPI_TalonFX ShooterR = new WPI_TalonFX(14);
  MotorControllerGroup shooterMotors = new MotorControllerGroup(ShooterL, ShooterR);

  // Variables to store shooter velocity and position
  double vel = 0.0;
  boolean pos = false;

  /**
   * Constructor
   */
  public ShooterSubsystem()
  {
    // TODO: initialize motor controllers and solenoid here!
    if(controller.getRawButton(8))
    {
      vel = 4000;
      shooterSolenoid.set(true);
      ShooterL.set(ControlMode.Velocity, vel);
      ShooterR.set(ControlMode.Velocity, vel);
      
    }else if(controller.getRawButton(7))
    {
      vel = 8500;
      ShooterL.set(ControlMode.Velocity, vel);
      ShooterR.set(ControlMode.Velocity, vel);
    }else
    {
shooterSolenoid.set(false);
shooterMotors.set(0);
    }
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    // TODO: write shooter velocity and hood position values here
    vel= 0;
    pos = false;
  }

  @Override
  public void simulationPeriodic()
  {
    // This method will be called once per scheduler run during simulation
  }

  public void setVel(double v)
  {
    vel = v;
  }

  public void setHood(boolean p)
  {
    pos = p;
  }

}
