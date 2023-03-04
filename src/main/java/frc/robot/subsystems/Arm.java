// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  //Dois motores, um de  cada lado 
  CANSparkMax  motorLeft = new CANSparkMax(ArmConstants.kArmLeftMotor,MotorType.kBrushless);
  CANSparkMax  motorRight = new CANSparkMax (ArmConstants.kArmRighrMotor,MotorType.kBrushless);
  
  //dois encoders, um de cada motor
  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;
  /** Creates a new arm. */
  public Arm() {
    //Limpo qualquer configuração  inicial dos modulos
    motorLeft.restoreFactoryDefaults();
    motorRight.restoreFactoryDefaults();

    //Configuro para  que o  motor se mantenha estatico quando em 0
    motorLeft.setIdleMode(IdleMode.kBrake);
    motorRight.setIdleMode(IdleMode.kBrake);
    //Configuro a rampa de aceleração para evitar picos de corrente
    motorLeft.setOpenLoopRampRate(ArmConstants.kRampRate);
    motorRight.setOpenLoopRampRate(ArmConstants.kRampRate);

    //Inverto o motor da esquerda para que girem juntos
    motorLeft.setInverted(true);

    //Associo os encoders, seto a razão de 1 volta e zero os mesmos
    leftEncoder = motorLeft.getEncoder();
    rightEncoder = motorRight.getEncoder();
    leftEncoder.setPositionConversionFactor(1);
    rightEncoder.setPositionConversionFactor(1);
    resetEncoder();
    
    
  }
  //Função principal que movimenta o braço para frente(+) e  para tras(-)
  public void setMotorPower(double forward) {
    SmartDashboard.putNumber("Braco Potencia (%)", forward * 100.0);

    motorRight.set(forward);
    motorLeft.set(forward);
    
  }

  //Reseta os valores dos encoders, para ter a referencia atual
  public void resetEncoder(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
  
  //Função  que captura  os encoders, fazendo uma media dos dois lados e dividindo pela redução
  public double getEncoder(){
    return ((rightEncoder.getPosition()+leftEncoder.getPosition())/2/ArmConstants.kGearRatio);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Braco Encoder", getEncoder());
    if(getEncoder()>ArmConstants.kMaxForwardTicks || getEncoder()<1){
      setMotorPower(0);
    }
  }


}
