// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AuxiliarIntakeConstants;
/*
 * Intake auxiliar, fica embaixo do robo
 */
public class AuxiliarIntake extends SubsystemBase {
  //Define o motor que foi usado
  //Um snowblower com um VictorSPX
  VictorSPX motor = new VictorSPX(AuxiliarIntakeConstants.kIntakeMotor);

  //Define um encoder para conseguir definir quando o intake baixou ou subiu
  Encoder encoder = new Encoder(AuxiliarIntakeConstants.kEncoderA, AuxiliarIntakeConstants.kEncoderB);
  public AuxiliarIntake() {
    motor.configFactoryDefault();
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Coast);
    encoder.setDistancePerPulse(1);
    resetEncoder();
  }
  public void resetEncoder(){
    encoder.reset();
  }

  public int getEncoder(){
    return encoder.get();
  }
  public void setMotor(double power){
    motor.set(ControlMode.PercentOutput,power);
  }
  public void setIntake(boolean forward){
    if(getEncoder()<100 && forward){
      motor.set(ControlMode.PercentOutput,AuxiliarIntakeConstants.kMotorPower);
    }
    else if(getEncoder()>200 && !forward){
      motor.set(ControlMode.PercentOutput,AuxiliarIntakeConstants.kMotorPower*-1);
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*if(getEncoder()>AuxiliarIntakeConstants.kPulsesDown){
      motor.set(ControlMode.PercentOutput,0);
    }
    if(getEncoder()<20){
      motor.set(ControlMode.PercentOutput,0);
      resetEncoder();
    }*/
    SmartDashboard.putNumber("Intake Auxiliar",getEncoder());
  }
}
