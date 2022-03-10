// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.Timer;

// must create + complete drive commands to implement 

// import frc.robot.commands.TeleopDriveCommand;

public class DrivetrainSubsystem extends SubsystemBase {
  // Wait I think this is wrong. FX are the new motors on the top and SRX are the old one on the bottom
  private TalonSRX leftDriveCim1, leftDriveCim2, rightDriveCim1, rightDriveCim2;
  private TalonFX leftDriveFalcon, rightDriveFalcon;
  private Timer m_timer;

  

  // Variables for left and right powers
  double lPwr = 0.0;
  double rPwr = 0.0;
  
  /**
   * Constructor.
   */
  public DrivetrainSubsystem() {
    m_timer = new Timer();
    // TODO: initialize drivetrain motor controllers
    // NOTE: Init is undefined in DrivetrainSubsystems POSSIBLY due to us not completing the drivetraincommands class - CHARLES
    leftDriveCim1 = init2(Constants.Drivetrain.leftDriveCim1_ID);
    leftDriveFalcon = init(Constants.Drivetrain.leftDriveFalcon_ID);
    leftDriveCim2 = init2(Constants.Drivetrain.leftDriveCim1_ID);
    
    rightDriveCim1 = init2(Constants.Drivetrain.leftDriveCim1_ID);
    rightDriveFalcon = init(Constants.Drivetrain.leftDriveFalcon_ID);
    rightDriveCim2 = init2(Constants.Drivetrain.leftDriveCim1_ID);

    // Talon specific setups
    leftDriveCim1.set(ControlMode.PercentOutput, 1.0);
    leftDriveFalcon.set(ControlMode.Follower, Constants.Drivetrain.leftDriveCim1_ID);
    leftDriveCim2.set(ControlMode.Follower, Constants.Drivetrain.leftDriveCim2_ID);

    rightDriveCim1.set(ControlMode.PercentOutput, 1.0);
    rightDriveFalcon.set(ControlMode.Follower, Constants.Drivetrain.rightDriveCim1_ID);
    rightDriveCim2.set(ControlMode.Follower, Constants.Drivetrain.rightDriveCim2_ID);

  }

  /**  Tank Drive configurations
   *  P.S. this is a comment...
   * @param l
   * @param r
   */

  public void setTank(double l, double r) {
    leftDriveCim1.set(ControlMode.PercentOutput, l);
    leftDriveFalcon.set(ControlMode.Follower, Constants.Drivetrain.leftDriveCim1_ID);
    leftDriveCim2.set(ControlMode.Follower, Constants.Drivetrain.leftDriveCim2_ID);

    rightDriveCim1.set(ControlMode.PercentOutput, -r);
    rightDriveFalcon.set(ControlMode.Follower, Constants.Drivetrain.rightDriveCim1_ID);
    rightDriveCim2.set(ControlMode.Follower, Constants.Drivetrain.rightDriveCim2_ID);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO: implement tank drive logic here!
  }

  public TalonFX init(int id) {
    TalonFX tFX = new TalonFX(id);

    tFX.configFactoryDefault();

    SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 60, 65, 3);
    tFX.configSupplyCurrentLimit(supplyCurrentLimit);

    tFX.setNeutralMode(NeutralMode.Coast);
    
    // Do common talon initialization stuff here.


    return tFX;

  }
 
  public TalonSRX init2(int id) {
    TalonSRX tSRX = new TalonSRX(id);

    tSRX.configFactoryDefault();

    SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 60, 65, 3);
    tSRX.configSupplyCurrentLimit(supplyCurrentLimit);

    tSRX.setNeutralMode(NeutralMode.Coast);
    // Do common talon initialization stuff here.


    return tSRX;

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Sets left and right motor powers
   */
  public void drive(double l, double r) {
    lPwr = l;
    rPwr = r;
    setTank(l, r);
  }

  // Simple autonomous drive command
  public void setAuto(double l, double r, double t) {
    m_timer.reset();
    m_timer.start();
    if (m_timer.get() < t) {
      setTank(l, r);
    } else {
      setTank(0, 0);
    }

  }
}
