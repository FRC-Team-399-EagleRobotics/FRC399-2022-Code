package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase; 

public class AutonomousIntake extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private IntakeSubsystem m_aintake;
    private double iPwr, t;
    private boolean iPos, out;
    Timer timer = new Timer();
    
    boolean isFinished = false;

    public AutonomousIntake(IntakeSubsystem m_aintake, double iPwr, boolean iPos, double t, boolean out) {
        this.m_aintake = m_aintake;
        this.iPwr = iPwr;
        this.iPos = iPos;
        this.t = t;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_aintake);
      }
      @Override
      public void initialize() {
        timer.reset();
        timer.start();
      }

      @Override
      public void execute() { // TODO Uness
        if (timer.get() < t) {
        if(out == false){
            m_aintake.extend();
            m_aintake.setPwr(1);
        }else if(out == true){

            m_aintake.extend();
            m_aintake.setPwr(-1);
        }
        }else{
            m_aintake.setPwr(0);
            m_aintake.retract();
        }
      }
      @Override
      public void end(boolean interrupted)
      {
        m_aintake.retract();
        m_aintake.setPwr(0);
      }

      @Override
      public boolean isFinished() {
        return isFinished;
      }
      
}
