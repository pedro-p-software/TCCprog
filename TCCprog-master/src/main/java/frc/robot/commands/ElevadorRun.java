// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevador;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevadorRun extends Command {
  double target=0;    
  
  Elevador elevador;
  PS4Controller controller;
  double POV = controller.getPOV();

  private SparkMax elevMaster = new SparkMax(5, MotorType.kBrushless);
  private SparkMax elevSlave = new SparkMax(6, MotorType.kBrushless);

  PIDController elevPidController = new PIDController(0, 0, 0);
  /** Creates a new ElevadorRun. */

  public ElevadorRun(Elevador elevador, PS4Controller controller) {
    this.elevador = elevador;
    this.controller = controller;
    addRequirements(elevador);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    POV = controller.getPOV();

    if(POV == 0){
           target = 130; 
      }else if(POV==180){
          target = 0;
      }else if (POV == 90){
          target = 100;
      }else if (POV==270){
        target=75;
      }
      elevador.run(target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
