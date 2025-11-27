// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevador;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoElevador extends Command {
  /** Creates a new AutoElevador. */
  Elevador elevador;
  Timer timer = new Timer();
  double maxTime;
  double target; // L2

  public AutoElevador(Elevador elevador, double maxTime, double target) {
    this.maxTime=maxTime;
    this.elevador=elevador;
    this.target=target;
    addRequirements(elevador);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevador.elevGoToTarget(target);
    if(timer.get()>= 4){
      target=0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    target = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get()>=maxTime; // É dado um tempo limite pro elevador não descer enquanto o shooter shoota
  }
}
