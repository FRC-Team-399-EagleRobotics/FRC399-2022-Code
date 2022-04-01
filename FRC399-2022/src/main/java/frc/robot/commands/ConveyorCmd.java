package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.Constants.Controls;
import frc.robot.subsystems.ConveyorSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ConveyorCmd extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private ConveyorSubsystem m_conveyor;
    
      public ConveyorCmd(ConveyorSubsystem m_conveyor, double aPwr, double bPwr) {
        this.m_conveyor = m_conveyor;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_conveyor);
      }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Pressing Right trigger to store. TODO You might need to time this but test it first
        if (RobotContainer.operator.getRawButton(Controls.leftTrigger_ID)){
            m_conveyor.setPwr(1, -1);
        // Pressing Right bumper leads to spitting out ball
        } else if (RobotContainer.operator.getRawButton(Controls.leftBumper_ID)) {
            m_conveyor.setPwr(1, 1);
        // Press Left Trigger to shoot. Waiting for shooter for the meantime just going to run conveyor
        } else if (RobotContainer.operator.getRawButton(Controls.rightTrigger_ID)) {
            m_conveyor.setPwr(-1, -1);
        }else if(RobotContainer.operator.getRawButton(Controls.rightBumper_ID)){
            m_conveyor.setPwr(1, -1);
        }
        // Else ends may not require timer do new button inputs or it may be a problem 
        else {
            m_conveyor.setPwr(0, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_conveyor.endConveyor();

    }

    @Override
    public boolean isFinished() {
        return false;
    }

 
}
