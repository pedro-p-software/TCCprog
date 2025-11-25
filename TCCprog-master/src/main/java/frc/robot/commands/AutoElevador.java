package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevador;

public class AutoElevador extends Command {
    Timer timer=new Timer();
    double maxTime;
    

    Elevador elevador;
    double target=30;
    /** Creates a new Atirar. */
    public AutoElevador(Elevador elevador, double maxTime) {
      this.elevador=elevador;
      this.maxTime=maxTime;
      addRequirements(elevador);
      // Use addRequirements() here to declare subsystem dependencies.
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        elevador.resetEncoders();
        timer.restart();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      elevador.elevGoToTarget(target);
      
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      elevador.elevGoToTarget(0);
      timer.stop();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        
      return timer.get() >= maxTime;
    }
  }
  
