// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase 
{
  // Calling out motors
  private TalonSRX ClimberL;
  private TalonSRX ClimberR;
  private MotorControllerGroup ClimbMotors;

  double cPWR = 0;

  ClimberL = new TalonSRX(Constants.Climber.leftClimberCim1_ID);
  ClimberR = new TalonSRX(Constants.Climber.rightClimberCim1_ID);
  ClimbMotors = new MotorControllerGroup(ClimberL, ClimberR);
  /** Creates a new ExampleSubsystem. */
  public ClimberSubsystem() 
  {
    // IDK if this is the right way got from drivetrain. 
    // Suppose this is for setting the motor variable 
    // Idk why its red or where you get init

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
  private void ClimberUp (double C)
  {
    cPWR = C;
    ClimbMotors.set(1);
  }
  private void ClimberDown (double C)
  {
    cPWR = C;
    ClimbMotors.set(-1);
  }
}
