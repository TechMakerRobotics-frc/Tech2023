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
  CANSparkMax  motorLeft = new CANSparkMax(ArmConstants.kArmLeftMotor,MotorType.kBrushless);
  CANSparkMax  motorRight = new CANSparkMax (ArmConstants.kArmRighrMotor,MotorType.kBrushless);
 
  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;
  /** Creates a new arm. */
  public Arm() {
    motorLeft.restoreFactoryDefaults();
    motorRight.restoreFactoryDefaults();
    motorLeft.setIdleMode(IdleMode.kBrake);
    motorRight.setIdleMode(IdleMode.kBrake);
    motorLeft.setOpenLoopRampRate(ArmConstants.kRampRate);
    motorRight.setOpenLoopRampRate(ArmConstants.kRampRate);
    motorLeft.setInverted(true);
    leftEncoder = motorLeft.getEncoder();
    rightEncoder = motorRight.getEncoder();
    leftEncoder.setPositionConversionFactor(1);
    rightEncoder.setPositionConversionFactor(1);
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    
    
  }
  public void setMotorPower(double forward) {
    SmartDashboard.putNumber("Braco Potencia (%)", forward * 100.0);


    // see note above in robotInit about commenting these out one by one to set
    // directions.
    motorRight.set(forward);
    motorLeft.set(forward);
    
  }
  public void resetEncoder(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
  
  public double getRightEncoder(){
    return rightEncoder.getPosition();
  }
  public double getLeftEncoder(){
    return leftEncoder.getPosition();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Braco Encoder Esquerda", getLeftEncoder());
    SmartDashboard.putNumber("Braco Encoder Direita", getRightEncoder());
    if(getLeftEncoder()>ArmConstants.kMaxForwardTicks || getRightEncoder()>ArmConstants.kMaxForwardTicks ||
    getLeftEncoder()<1 || getRightEncoder()<1){
      setMotorPower(0);
    }
  }


}
