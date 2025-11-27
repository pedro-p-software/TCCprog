// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Elevador;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoL3 extends ParallelCommandGroup {
  /** Creates a new AutoL3. */
  public AutoL3(Elevador elevador, Shooter shooter) {
    
    addCommands(
      new AutoElevador(elevador, 7, 30), 
      new WaitnShoot(4, shooter));
  }
}
/* Aqui o elevador liga e como o comando é paralelo, executa o do shooter tambem. Enquanto o elevador sobe, o shooter executa o Wait (estimado 4 segundos)
 * aí quando acabam os 4 seg o shooter shoota e são dados mais 3 segundos pra isso acontecer. Após, o elevador desce.
 */