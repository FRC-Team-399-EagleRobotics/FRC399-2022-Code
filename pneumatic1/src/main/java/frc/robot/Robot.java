// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Darian - commented out unused code, kept it incase we use a double solenoid
//import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Compressor;

/**
 * This is a sample program showing the use of the solenoid classes during operator control. Three
 * buttons from a joystick will be used to control two solenoids: One button to control the position
 * of a single solenoid and the other two buttons to control a double solenoid. Single solenoids can
 * either be on or off, such that the air diverted through them goes through either one channel or
 * the other. Double solenoids have three states: Off, Forward, and Reverse. Forward and Reverse
 * divert the air through the two channels and correspond to the on and off of a single solenoid,
 * but a double solenoid can also be "off", where the solenoid will remain in its default power off
 * state. Additionally, double solenoids take up two channels on your PCM whereas single solenoids
 * only take a single channel.
 */
public class Robot extends TimedRobot {
  private final Joystick m_stick = new Joystick(2);

      boolean toggleOn = false;
      boolean togglePressed = false;
      boolean toggleOn2 = false;
      boolean togglePressed2 = false;

  // Solenoid corresponds to a single solenoid.
  Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  private final Solenoid m_solenoid2 = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

  // DoubleSolenoid corresponds to a double solenoid.
  /*private final DoubleSolenoid m_doubleSolenoid =
     new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);*/

  /*private static final int kSolenoidButton = 4;
  private static final int kDoubleSolenoidForward = 2;
  private static final int kDoubleSolenoidReverse = 3;*/

  // created toggle switch to activate solenoid
  public void updateToggle()
  {
    if(m_stick.getRawButton(3)){
      if(!togglePressed){
        toggleOn = !toggleOn;
        togglePressed = true;
      }
    }else{
      togglePressed = false;}
    }
  public void updateToggle2()
    {
      if(m_stick.getRawButton(4)){
        if(!togglePressed2){
          toggleOn2 = !toggleOn2;
          togglePressed2 = true;
        }
      }else{
        togglePressed2 = false;}
      }
    
  
  @Override
  public void teleopPeriodic() {
    updateToggle();
    // toggle switch for solenoid valve
    if(toggleOn){
      m_solenoid.set(true);
    }else{
      m_solenoid.set(false);
    }

    updateToggle2();
    // toggle switch for second solenoid valve
    if(toggleOn2){
      m_solenoid2.set(true);
    }else{
      m_solenoid2.set(false);
    }

    /*
     * In order to set the double solenoid, if just one button
     * is pressed, set the solenoid to correspond to that button.
     * If both are pressed, set the solenoid will be set to Forwards.
     */
    if (m_stick.getRawButton(1)) {
      pcmCompressor.enableDigital();
    } else if (m_stick.getRawButton(2)) {
      pcmCompressor.disable();}
  }
}
    

  
