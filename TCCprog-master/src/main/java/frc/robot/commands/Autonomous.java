package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevador;
import frc.robot.subsystems.Shooter;


public class Autonomous extends SequentialCommandGroup {

  public Autonomous(DriveTrain driveTrain, Elevador elevador, Shooter shooter){
    addCommands(
        new AutoFT(driveTrain, 2), 
        new AutoElevador(elevador, 7, 30));
        new AutoFT(driveTrain, -4);
        new AutoUnAtirar(shooter);
        new AutoFT(driveTrain, 4);
        new AutoElevador(elevador, 5, 20);

      
  }
}