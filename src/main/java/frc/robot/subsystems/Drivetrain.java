// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
public class Drivetrain extends SubsystemBase {
  private CANSparkMax  motorLeftFront = new CANSparkMax(DrivetrainConstants.kMotorLeftFront,MotorType.kBrushless);
  private CANSparkMax  motorLeftRear = new CANSparkMax (DrivetrainConstants.kMotorLeftRear,MotorType.kBrushless);
  private CANSparkMax  motorRightFront = new CANSparkMax (DrivetrainConstants.kMotorRightFront,MotorType.kBrushless);
  private CANSparkMax  motorRightRear = new CANSparkMax (DrivetrainConstants.kMotorRightRear,MotorType.kBrushless);

  private RelativeEncoder leftEncoder1, leftEncoder2;
  private RelativeEncoder rightEncoder1, rightEncoder2;

  private AHRS navX;
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    motorLeftFront.restoreFactoryDefaults();
    motorLeftRear.restoreFactoryDefaults();
    motorRightFront.restoreFactoryDefaults();
    motorRightRear.restoreFactoryDefaults();
    motorLeftFront.setIdleMode(IdleMode.kCoast);
    motorLeftRear.setIdleMode(IdleMode.kCoast);
    motorRightFront.setIdleMode(IdleMode.kCoast);
    motorRightRear.setIdleMode(IdleMode.kCoast);
    motorLeftFront.setInverted(true);
    motorLeftRear.setInverted(true);

    motorLeftFront.setOpenLoopRampRate(DrivetrainConstants.kRampRate);
    motorLeftRear.setOpenLoopRampRate(DrivetrainConstants.kRampRate);
    motorRightFront.setOpenLoopRampRate(DrivetrainConstants.kRampRate);
    motorRightRear.setOpenLoopRampRate(DrivetrainConstants.kRampRate);

    motorLeftFront.setSmartCurrentLimit(DrivetrainConstants.kMotorCurrentLimit);
    motorLeftRear.setSmartCurrentLimit(DrivetrainConstants.kMotorCurrentLimit);
    motorRightFront.setSmartCurrentLimit(DrivetrainConstants.kMotorCurrentLimit);
    motorRightRear.setSmartCurrentLimit(DrivetrainConstants.kMotorCurrentLimit);

    //motorLeftFront.follow(motorLeftRear);
    //motorRightFront.follow(motorRightRear);

    leftEncoder1 = motorLeftRear.getEncoder();
    rightEncoder1 = motorRightRear.getEncoder();
    leftEncoder1.setPositionConversionFactor(1);
    rightEncoder1.setPositionConversionFactor(1);
    leftEncoder1.setPosition(0);
    rightEncoder1.setPosition(0);
    leftEncoder2 = motorLeftFront.getEncoder();
    rightEncoder2 = motorRightFront.getEncoder();
    leftEncoder2.setPositionConversionFactor(1);
    rightEncoder2.setPositionConversionFactor(1);
    leftEncoder2.setPosition(0);
    rightEncoder2.setPosition(0);

    //navX = new AHRS(DrivetrainConstants.NAVX_PORT);
    resetYaw();


    
  }
  public void setDriveMotors(double forward, double turn) {
    SmartDashboard.putNumber("Potencia Frente (%)", forward * 100.0);
    SmartDashboard.putNumber("Potencia Curva (%)", turn * 100.0);

    /*
     * positive turn = counter clockwise, so the left side goes backwards
     */
    double left = forward - turn;
    double right = forward + turn;

    SmartDashboard.putNumber("Potencia Esquerda (%)", left * 100.0);
    SmartDashboard.putNumber("Potencia Direita (%)", right * 100.0);

    // see note above in robotInit about commenting these out one by one to set
    // directions.
    motorLeftRear.set(left);
    motorLeftFront.set(left);
    motorRightRear.set(right);
    motorRightFront.set(right);
  }
  public void resetEncoder(){
    leftEncoder1.setPosition(0);
    leftEncoder2.setPosition(0);
    rightEncoder1.setPosition(0);
    rightEncoder2.setPosition(0);
  }
  public void setTankMotors(double left, double right) {

    SmartDashboard.putNumber("Potencia Esquerda (%)", left * 100.0);
    SmartDashboard.putNumber("Potencia Direita (%)", right * 100.0);

    // see note above in robotInit about commenting these out one by one to set
    // directions.
    motorLeftRear.set(left);
    motorLeftFront.set(left);
    motorRightRear.set(right);
    motorRightFront.set(right);
  }
  public double getRightEncoder(){
    return ((rightEncoder1.getPosition()+rightEncoder2.getPosition())/2/DrivetrainConstants.kGearboxRatio)*DrivetrainConstants.kWheelDistance;
  }
  public double getLeftEncoder(){
    return ((leftEncoder1.getPosition()+leftEncoder2.getPosition())/2/DrivetrainConstants.kGearboxRatio)*DrivetrainConstants.kWheelDistance;
  }
  public double getYaw() {
		return 0;//navX.getYaw();
	}
  public void resetYaw(){
    //navX.reset();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Distancia Esquerda", getLeftEncoder());
    SmartDashboard.putNumber("Distancia Direita", getRightEncoder());
    SmartDashboard.putNumber("Giro", getYaw());
  }
}
