package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCmd extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private ShooterSubsystem m_shooter;

    public ShooterCmd(ShooterSubsystem subsystem) {
        m_shooter = subsystem;
        //Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
      }

    @Override
    public void initialize() {

    }

  @Override
  public void execute() {
    if(RobotContainer.operator.getRawButton(8))
    {
        m_shooter.midShot();
    }else if(RobotContainer.operator.getRawButton(7))
    {
        m_shooter.nearShot();
    }else
    {
     m_shooter.setHood(false);
    }
}

    @Override
    public void end(boolean interrupted) {
        m_shooter.setHood(false);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
