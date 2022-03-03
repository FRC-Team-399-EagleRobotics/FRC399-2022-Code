// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.Compressor;

public class ShooterSubsystem extends SubsystemBase {

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
  BangBangController BANGshooter = new BangBangController();
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(10, 20);

  // Variables to store shooter velocity and position
  double vel = 0.0;
  boolean pos = false;

  /**
   * Constructor
   */
  public ShooterSubsystem()
  {
    if(controller.getRawButton(8))
    {
      setVel(4000);
      setHood(true);
    }else if(controller.getRawButton(7))
    {
      setVel(8500);
    }else
    {
setHood(false);
shooterMotors.set(0);
    }
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic()
  {
    // This method will be called once per scheduler run during simulation
  }

  public void setVel(double v)
  {
    vel = v;
    ShooterL.set(ControlMode.Velocity, v);
    ShooterR.set(ControlMode.Velocity, v);
    shooterMotors.set(BANGshooter.calculate(ShooterL.getSelectedSensorVelocity(), v) + 0.9 * feedforward.calculate(v));
  }

  public void setHood(boolean p)
  {
    pos = p;
    shooterSolenoid.set(p);
  }

}
