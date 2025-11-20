// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
   private WPI_TalonSRX rightMaster = new WPI_TalonSRX(4);
    private WPI_TalonSRX rightSlave = new WPI_TalonSRX(1);
    private WPI_TalonSRX leftMaster = new WPI_TalonSRX(2);
    private WPI_TalonSRX leftSlave = new WPI_TalonSRX(3);

    private DifferentialDrive diffDrive = new DifferentialDrive(leftMaster, rightMaster);

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    rightSlave.follow(rightMaster);
    leftSlave.follow(leftMaster);

    leftMaster.setInverted(false);
    leftSlave.setInverted(false);
    rightMaster.setInverted(false);
    rightSlave.setInverted(false);

  }

  public void Drive(double speed, double rotation){
    diffDrive.arcadeDrive(speed, rotation);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run. sure buddy
  }
}
