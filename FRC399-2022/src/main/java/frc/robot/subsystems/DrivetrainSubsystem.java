// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  // TODO: Instantiate drivetrain motor controllers


  // Variables for left and right powers
  double lPwr = 0.0;
  double rPwr = 0.0;
  
  /**
   * Constructor.
   */
  public DrivetrainSubsystem() {
    // TODO: initialize drivetrain motor controllers
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // TODO: implement tank drive logic here!
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Sets left and right motor powers
   */
  public void set(double l, double r) {
    lPwr = l;
    rPwr = r;
  }
}
