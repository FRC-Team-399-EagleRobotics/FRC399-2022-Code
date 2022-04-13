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

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;

// must create + complete drive commands to implement 

// import frc.robot.commands.TeleopDriveCommand;

public class DrivetrainSubsystem extends SubsystemBase {
  // Wait I think this is wrong. FX are the new motors on the top and SRX are the old one on the bottom
  private TalonSRX leftDriveCim1, leftDriveCim2, rightDriveCim1, rightDriveCim2;
  private TalonFX leftDriveFalcon, rightDriveFalcon;

  // These represent our regular encoder objects, which we would
  // create to use on a real robot.
  private Encoder m_leftEncoder = new Encoder(0, 1);
  private Encoder m_rightEncoder = new Encoder(2, 3);

  // These are our EncoderSim objects, which we will only use in
  // simulation. However, you do not need to comment out these
  // declarations when you are deploying code to the roboRIO.
  private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

  // Create our gyro object like we would on a real robot.
  private AnalogGyro m_gyro = new AnalogGyro(1);

  // Create the simulated gyro object, used for setting the gyro
  // angle. Like EncoderSim, this does not need to be commented out
  // when deploying code to the roboRIO.
  private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
  
  static final double KvLinear = 1.98;
  static final double KaLinear = 0.2;
  static final double KvAngular = 1.5;
  static final double KaAngular = 0.3;
  static final int kWheelRadius = 3;
  static final double kEncoderResolution = 21;
  private PWMSparkMax m_leftMotor = new PWMSparkMax(0);
private PWMSparkMax m_rightMotor = new PWMSparkMax(1);


// Create the simulation model of our drivetrain.
DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
  DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
  7.29,                    // 7.29:1 gearing reduction.
  7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
  60.0,                    // The mass of the robot is 60 kg.
  Units.inchesToMeters(3), // The robot uses 3" radius wheels.
  0.7112,                  // The track width is 0.7112 meters.

  // The standard deviations for measurement noise:
  // x and y:          0.001 m
  // heading:          0.001 rad
  // l and r velocity: 0.1   m/s
  // l and r position: 0.005 m
  VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
  
  private Field2d m_field = new Field2d();
  // Creating my odometry object. Here,
// our starting pose is 5 meters along the long end of the field and in the
// center of the field along the short end, facing forward.
DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
  getGyroHeading(), new Pose2d(5.0, 13.5, new Rotation2d()));

 /* private Encoder leftEncoder = new Encoder(//
            Constants.Drivetrain.LeftDriveCim1EnC_A, Constants.Drivetrain.LeftDriveCim1EnC_B);

  private Encoder leftEncoder2 = new Encoder(//
            Constants.Drivetrain.LeftDriveCim2EnC_A, Constants.Drivetrain.LeftDriveCim2EnC_B);

  private Encoder leftEncoder3 = new Encoder(//
            Constants.Drivetrain.LeftDriveFalconEnC_A, Constants.Drivetrain.LeftDriveFalconEnC_B);

  private Encoder rightEncoder = new Encoder(//
            Constants.Drivetrain.LeftDriveCim1EnC_A, Constants.Drivetrain.LeftDriveCim1EnC_B);

  private Encoder rightEncoder2 = new Encoder(//
            Constants.Drivetrain.LeftDriveCim2EnC_A, Constants.Drivetrain.LeftDriveCim2EnC_B);

  private Encoder rightEncoder3 = new Encoder(//
            Constants.Drivetrain.LeftDriveFalconEnC_A, Constants.Drivetrain.LeftDriveFalconEnC_B);*/
  
  // Variables for left and right powers
  double lPwr = 0.0;
  double rPwr = 0.0;
  
  /**
   * Constructor.
   */
  public DrivetrainSubsystem() {
    // TODO: initialize drivetrain motor controllers
    // NOTE: Init is undefined in DrivetrainSubsystems POSSIBLY due to us not completing the drivetraincommands class - CHARLES
    leftDriveCim1 = init2(Constants.Drivetrain.leftDriveCim1_ID);
    leftDriveFalcon = init(Constants.Drivetrain.leftDriveFalcon_ID);
    leftDriveCim2 = init2(Constants.Drivetrain.leftDriveCim2_ID);
    
    rightDriveCim1 = init2(Constants.Drivetrain.rightDriveCim1_ID);
    rightDriveFalcon = init(Constants.Drivetrain.rightDriveFalcon_ID);
    rightDriveCim2 = init2(Constants.Drivetrain.rightDriveCim2_ID);

    // Talon specific setups
    leftDriveCim1.set(ControlMode.PercentOutput, 1.0);
    leftDriveFalcon.set(ControlMode.PercentOutput, 1.0);
    leftDriveCim2.set(ControlMode.PercentOutput, 1.0);

    rightDriveCim1.set(ControlMode.PercentOutput, 1.0);
    rightDriveFalcon.set(ControlMode.PercentOutput, 1.0);
    rightDriveCim2.set(ControlMode.PercentOutput, 1.0);

    m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    SmartDashboard.putData("Field", m_field);
  }

  //public double getEncoderMeters() {
    //return (leftEncoder.get() + -rightEncoder.get()) / 2 * Drivetrain.kEncoderTick2Meter;
//}


  private Rotation2d getGyroHeading() {
    return null;
  }

  /**  Tank Drive configurations
   *  P.S. this is a comment...
   * @param l
   * @param r
   */

  public void setTank(double l, double r) {
    leftDriveCim1.set(ControlMode.PercentOutput, l);
    leftDriveFalcon.set(ControlMode.PercentOutput, l);
    leftDriveCim2.set(ControlMode.PercentOutput, l);

    rightDriveCim1.set(ControlMode.PercentOutput, -r);
    rightDriveFalcon.set(ControlMode.PercentOutput, -r);
    rightDriveCim2.set(ControlMode.PercentOutput, -r);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO: implement tank drive logic here!
      // This will get the simulated sensor readings that we set
  // in the previous article while in simulation, but will use
  // real values on the robot itself.
  m_odometry.update(m_gyro.getRotation2d(),
  m_leftEncoder.getDistance(),
  m_rightEncoder.getDistance());
m_field.setRobotPose(m_odometry.getPoseMeters());
}

  public TalonFX init(int id) {
    TalonFX tFX = new TalonFX(id);

    tFX.configFactoryDefault();

    SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 60, 65, 3);
    tFX.configSupplyCurrentLimit(supplyCurrentLimit);

    tFX.setNeutralMode(NeutralMode.Brake);
    
    // Do common talon initialization stuff here.


    return tFX;

  }
 
  public TalonSRX init2(int id) {
    TalonSRX tSRX = new TalonSRX(id);

    tSRX.configFactoryDefault();

    SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 60, 65, 3);
    tSRX.configSupplyCurrentLimit(supplyCurrentLimit);

    tSRX.setNeutralMode(NeutralMode.Brake);
    // Do common talon initialization stuff here.


    return tSRX;

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
      // Set the inputs to the system. Note that we need to convert
  // the [-1, 1] PWM signal to voltage by multiplying it by the
  // robot controller voltage.
  m_driveSim.setInputs(m_leftMotor.get() * RobotController.getInputVoltage(),
  m_rightMotor.get() * RobotController.getInputVoltage());

// Advance the model by 20 ms. Note that if you are running this
// subsystem in a separate thread or have changed the nominal timestep
// of TimedRobot, this value needs to match it.
m_driveSim.update(0.02);

// Update all of our sensors.
m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());

    
  }

  /**
   * Sets left and right motor powers
   */
  public void drive(double l, double r) {
    lPwr = l;
    rPwr = r;
    drive(l, r);
  }
}
