package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class AutonomousClimber extends CommandBase{
    private ClimberSubsystem m_aclimber;
    private double cPwr, t;
    private boolean cPos;
    Timer timer = new Timer();
    boolean isFinished = false;

    public AutonomousClimber(ClimberSubsystem m_aclimber, double cPwr, boolean cPos, double t) {
        this.m_aclimber = m_aclimber;
        this.cPwr = cPwr;
        this.cPos = cPos;
        this.t = t;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_aclimber);


    }
        @Override
        public void initialize() {
          timer.reset();
          timer.start();
        }

        @Override
        public void execute() { 
         m_aclimber.setPos(cPos);
          if (timer.get() < t) {
            m_aclimber.climberControl(cPwr);
         } else {
            m_aclimber.setPos(false);
            m_aclimber.climberControl(0); 
         }

        }
        @Override
        public void end(boolean interrupted)
        {
            m_aclimber.setPos(false);
            m_aclimber.climberControl(0);
        }
  
        @Override
        public boolean isFinished() {
          return isFinished;
        }
        
  }
