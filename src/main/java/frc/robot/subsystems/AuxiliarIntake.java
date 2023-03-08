// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AuxiliarIntakeConstants;
/*
 * Intake auxiliar, fica embaixo do robo
 */
public class AuxiliarIntake extends SubsystemBase {
  //Define o motor que foi usado
  //Um snowblower com um VictorSPX
  VictorSPX motor = new VictorSPX(AuxiliarIntakeConstants.kIntakeMotor);

  public AuxiliarIntake() {
    motor.configFactoryDefault();
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Coast);
  }
  public void setMotor(double power){
    motor.set(ControlMode.PercentOutput,power);
  }
  public void setIntake(double forward){
      motor.set(ControlMode.PercentOutput,forward);
  }
  @Override
  public void periodic() {
    
  }
}
