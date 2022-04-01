package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonomousShooter extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private ShooterSubsystem m_ashooter;
    private double vel, t;
    private boolean pos;
    Timer timer = new Timer();
    
    boolean isFinished = false;

    public AutonomousShooter(ShooterSubsystem m_ashooter, double vel, boolean pos, double t) {
        this.m_ashooter = m_ashooter;
        this.vel = vel;
        this.pos = pos;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_ashooter);
      }
      @Override
      public void initialize() {
        timer.reset();
        timer.start();
      }

      @Override
      public void execute() {
        if (timer.get() < t) {
            m_ashooter.setVel(0.3);
      } else {
            m_ashooter.setVel(0);
      }
        }
      @Override
      public void end(boolean interrupted)
      {
        m_ashooter.endShooter();
      }

      @Override
      public boolean isFinished() {
        return isFinished;
      }
}
