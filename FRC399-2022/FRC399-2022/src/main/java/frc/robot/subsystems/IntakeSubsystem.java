// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  // TODO: instantiate intake motor controller and solenoid here!


  // Variables to store state of intake 
  double iPwr = 0.0;
  boolean iPos = false;

  /**
   * Constructor
   */
  public IntakeSubsystem() {
    // TODO: Initialize intake motor controller and solenoid

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO: write iPwr and iPos to motor controller and solenoid outputs
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setPwr(double i) {
    iPwr = i;
  }

  public void setPos(boolean p) {
    iPos = p;
  }
}
