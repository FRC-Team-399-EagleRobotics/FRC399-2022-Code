// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorSubsystem extends SubsystemBase {
  // TODO: instantiate conveyor motors
  private TalonSRX topConveyor, bottomConveyor;
  
  // Variables for motor power
  double aPwr = 0.0;
  double bPwr = 0.0;

  /**
   * Constructor.
   */
  public ConveyorSubsystem() {
    // Initialize conveyor motors
    topConveyor = new TalonSRX(Constants.Conveyor.topConveyor_ID);
    bottomConveyor = new TalonSRX(Constants.Conveyor.bottomConveyor_ID);
    topConveyor.setNeutralMode(NeutralMode.Brake);
    bottomConveyor.setNeutralMode(NeutralMode.Brake);

  }

  public void setConveyor(double a, double b) {
    topConveyor.set(ControlMode.PercentOutput, a);
    bottomConveyor.set(ControlMode.PercentOutput, b);
  }
  /**
   * Sets both conveyor motors
   */
  // Test speeds or time
  public void intake(){
    setPwr(0.25, -0.25);
  }
  public void store() {
    setPwr(0.8, -0.8);
  }

  public void spit(){
    setPwr(1, 1);
  }

  public void load() {
    setPwr(-1, -1);
  }

  public void endConveyor() {
    setPwr(0,0);
  }

  public void setPwr(double a, double b) {
    aPwr = a;
    bPwr = b;

    topConveyor.set(ControlMode.PercentOutput, a);
    bottomConveyor.set(ControlMode.PercentOutput, b);
  }
  
}
