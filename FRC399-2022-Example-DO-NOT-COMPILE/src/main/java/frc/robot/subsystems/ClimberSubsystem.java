// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  // Calling out motors
  private TalonSRX leftClimberCim1_ID, rightClimberCim1_ID;
  
  /** Creates a new ExampleSubsystem. */
  public ClimberSubsystem() {
    // IDK if this is the right way got from drivetrain. 
    // Suppose this is for setting the motor variable 
    // Idk why its red or where you get init
    leftClimberCim1_ID = init(Constants.Climber.leftClimberCim1_ID);
    rightClimberCim1_ID = init2(Constants.Climber.leftClimberCim1_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
