// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevador extends SubsystemBase {

  private double target;

  private SparkMax elevMaster = new SparkMax(5, MotorType.kBrushless);
  private SparkMax elevSlave = new SparkMax(6, MotorType.kBrushless);

  private AbsoluteEncoder masterEncoder;
  private AbsoluteEncoder slaveEncoder;

  private double posMaster = masterEncoder.getPosition();
  private double posSLave = -slaveEncoder.getPosition();

  private PIDController elevPidController = new PIDController(0.01, 0, 0); 

  /** Creates a new Elevador. */
  public Elevador() {
    var motorMasterConfig = new SparkMaxConfig();
    motorMasterConfig.idleMode(IdleMode.kBrake);
    motorMasterConfig.inverted(false);

    elevMaster.configure(motorMasterConfig, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    var motorSlaveConfig = new SparkMaxConfig();
    motorSlaveConfig.idleMode(IdleMode.kBrake);
    motorSlaveConfig.inverted(false);

    elevSlave.configure(motorSlaveConfig, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
    masterEncoder = elevMaster.getAbsoluteEncoder();
    slaveEncoder = elevSlave.getAbsoluteEncoder();

  }
public void run(double speed){
  elevMaster.set(speed);
  elevSlave.set(-speed);
}

  public void setTarget(double target){
    this.target = target;
  }

  public double getTarget(){
    return target;
  }

  public double getHeight(){
    double height = (posMaster + posSLave)/2;
    return height;
  }
  public void elevGoToTarget(double target){
    elevPidController.setSetpoint(target);
    double speedAtual = elevPidController.calculate(getHeight());
    double speed = MathUtil.clamp(speedAtual, -0.1, 0.3);
    run(speed);
  }
  @Override
  public void periodic() {
//    if(controller.getCrossButtonPressed()){
//      target = valorDoL2;
//    }else if(controller.getCircleButtonPressed()){
//      target = valorDoL3;
//    }else if(controller.getTriangleButtonPressed()){
//      target = valorDoL4;
//    }
SmartDashboard.putData("PID do Elevador",elevPidController);
SmartDashboard.putNumber("Encoders position", getHeight());
   
  }
}
