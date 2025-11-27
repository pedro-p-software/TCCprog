// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoRotate extends Command {
    DriveTrain driveTrain;
    double targetAngle;
  
  /** Creates a new Auto. */
  public AutoRotate(DriveTrain driveTrain, double targetAngle) {
    this.driveTrain = driveTrain;
    this.targetAngle=targetAngle;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetGyro();
    
    SmartDashboard.putString("Começou o rotate?", "Começou!");
    SmartDashboard.putString("Terminou o rotate?", "Não!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.driveTrainRotate(targetAngle);
    SmartDashboard.putNumber("target do rotate (graus)", targetAngle);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    driveTrain.Drive(0, 0);
    SmartDashboard.putString("Terminou?", "Terminou!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveTrain.rotateIsAtSetpoint();
  }
}