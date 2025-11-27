// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  double kConversaoTicksParaM= 0.00011688933603688586170451827431929;
  
  private WPI_TalonSRX leftSlave = new WPI_TalonSRX(1);//oi
  private WPI_TalonSRX rightSlave = new WPI_TalonSRX(2);
  private WPI_TalonSRX leftMaster = new WPI_TalonSRX(3);
  private WPI_TalonSRX rightMaster = new WPI_TalonSRX(4);
  
  private DifferentialDrive diffDrive = new DifferentialDrive(leftMaster, rightMaster);
  
  PIDController driveTrainPidController = new PIDController(0.5, 0, 0);
  
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    SmartDashboard.putData("PID driveTrain", driveTrainPidController);
    driveTrainPidController.setTolerance(0.1);

    rightSlave.follow(rightMaster);
    leftSlave.follow(leftMaster);
    
    leftMaster.setInverted(true);
    leftSlave.setInverted(true);
    rightMaster.setInverted(true);
    rightSlave.setInverted(false);
    
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
  }
  
  public void Drive(double speed, double rotation){
    diffDrive.arcadeDrive(speed, rotation);
  }
  
  public double getWheelPos(){
    double wheelPos = (leftMaster.getSelectedSensorPosition()*kConversaoTicksParaM  - rightMaster.getSelectedSensorPosition()*kConversaoTicksParaM)/2;
    return wheelPos;
  }
  
  public boolean isAtSetpoint(){
    return driveTrainPidController.atSetpoint();
  }
  
  
  public void resetEncoders(){
    rightMaster.setSelectedSensorPosition(0);
    leftMaster.setSelectedSensorPosition(0);
  }
  
  public void driveTrainGoToTarget(double target){
    driveTrainPidController.setSetpoint(target);
    double speedPorPosAtual = driveTrainPidController.calculate(getWheelPos());
    double speed = MathUtil.clamp(speedPorPosAtual, -0.75, 0.75);
    Drive(-speed, 0);
    SmartDashboard.putNumber("Speed do drivetrain", speed);
  }
  
  @Override
  public void periodic() {
    getWheelPos();
    SmartDashboard.putNumber("Coordenada",getWheelPos());
    SmartDashboard.putNumber("left encoder", leftMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("right encoder", rightMaster.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Ta no setpoint sera", driveTrainPidController.atSetpoint());
    // This method will be called once per scheduler run. sure buddy
  }
}
