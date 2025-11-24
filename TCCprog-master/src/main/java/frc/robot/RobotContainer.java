// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Atirar;
import frc.robot.commands.AutoFT;
import frc.robot.commands.Drive;
import frc.robot.commands.ElevadorRun;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevador;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
import frc.robot.subsystems.Shooter;
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  DriveTrain driveTrain = new DriveTrain();
  Elevador elevador = new Elevador();
  Shooter shooter = new Shooter();
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  PS4Controller controller = new PS4Controller(0);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

   
  private void configureBindings() {

    driveTrain.setDefaultCommand(new Drive(driveTrain, controller));
    
    elevador.setDefaultCommand(new ElevadorRun(elevador, controller));

    new JoystickButton(controller, 2).whileTrue(new Atirar(shooter));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new AutoFT(driveTrain, 0);
  }
}
