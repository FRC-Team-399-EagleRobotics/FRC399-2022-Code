package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.ConveyorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ConveyorCmd extends CommandBase {
    private ConveyorSubsystem m_conveyor;
    
      public ConveyorCmd(ConveyorSubsystem subsystem) {
        m_conveyor = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
      }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Pressing Right trigger to store. TODO You might need to time this but test it first
        if (RobotContainer.operator.getRawButton(8)){
            m_conveyor.store();
        // Pressing Right bumper leads to spitting out ball
        } else if (RobotContainer.operator.getRawButtonPressed(6)) {
            m_conveyor.spit();
        // Else ends may not require timer do new button inputs or it may be a problem
        } else {
            m_conveyor.endConveyor();
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
