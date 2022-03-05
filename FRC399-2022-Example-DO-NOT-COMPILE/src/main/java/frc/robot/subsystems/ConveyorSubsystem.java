// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConveyorSubsystem extends SubsystemBase {
  // TODO: instantiate conveyor motors

  
  // Variables for motor power
  double aPwr = 0.0;
  double bPwr = 0.0;

  /**
   * Constructor.
   */
  public ConveyorSubsystem() {
    // TODO: initialize conveyor motors


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //TODO: set motor controllers to aPwr and bPwr

    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Sets both conveyor motors
   */
  public void set(double a, double b) {
    aPwr = a;
    bPwr = b;
  }
}
