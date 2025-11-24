// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
   private WPI_TalonSRX rightMaster = new WPI_TalonSRX(4);
    private WPI_TalonSRX rightSlave = new WPI_TalonSRX(2);
    private WPI_TalonSRX leftMaster = new WPI_TalonSRX(3);
    private WPI_TalonSRX leftSlave = new WPI_TalonSRX(1);

    private DifferentialDrive diffDrive = new DifferentialDrive(leftMaster, rightMaster);

    PIDController driveTrainPidController;

    private RelativeEncoder rightEncoder;
    private RelativeEncoder leftEncoder;

    private double wheelPos;

    private double target;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    rightSlave.follow(rightMaster);
    leftSlave.follow(leftMaster);

    leftMaster.setInverted(true);
    leftSlave.setInverted(true);
    rightMaster.setInverted(true);
    rightSlave.setInverted(false);

  }

  public void Drive(double speed, double rotation){
    diffDrive.arcadeDrive(speed, rotation);
  }
 
  public double getWheelPos(){
    double wheelPos = (leftEncoder.getPosition() - rightEncoder.getPosition())/2;
    return wheelPos;
  }

  public boolean isAtSetpoint(){
    if(getWheelPos() == target){
      return true;
    }else{
      return false;
    }
  
  }


public void resetEncoders(){
  rightEncoder.setPosition(0);
  leftEncoder.setPosition(0);
}
  public void setTarget(double target){
    this.target = target;
  }
  public void driveTrainGoToTarget(double target){
    double speedPorPosAtual = driveTrainPidController.calculate(wheelPos);
    driveTrainPidController.setSetpoint(target-speedPorPosAtual);
    double speed = MathUtil.clamp(speedPorPosAtual, -0.1, 0.25);
    Drive(speed, 0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run. sure buddy
  }
}
