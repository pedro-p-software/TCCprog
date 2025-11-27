package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoShoot extends Command {
  /** Creates a new AutoShootL3. */
  double waitTime;
  Timer timer = new Timer();
  Shooter shooter;
  public AutoShoot(Shooter shooter, double waitTime) {
    this.waitTime=waitTime;
    this.shooter=shooter;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.Shoot(-0.75);  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.Shoot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get()>=waitTime;
  }
}
