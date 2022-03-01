// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  // TODO: instantiate shooter motor controller and hood solenoid here!



  // Variables to store shooter velocity and position
  double vel = 0.0;
  boolean pos = false;

  /**
   * Constructor
   */
  public ShooterSubsystem() {
    // TODO: initialize motor controllers and solenoid here!

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO: write shooter velocity and hood position values here
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setVel(double v) {
    vel = v;
  }

  public void setHood(boolean p) {
    pos = p;
  }

}
