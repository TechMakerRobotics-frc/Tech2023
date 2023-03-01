// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
public class Drivetrain extends SubsystemBase {
  CANSparkMax  motorLeftFront = new CANSparkMax(DrivetrainConstants.kMotorLeftFront,MotorType.kBrushless);
  CANSparkMax  motorLeftRear = new CANSparkMax (DrivetrainConstants.kMotorLeftRear,MotorType.kBrushless);
  CANSparkMax  motorRightFront = new CANSparkMax (DrivetrainConstants.kMotorRightFront,MotorType.kBrushless);
  CANSparkMax  motorRightRear = new CANSparkMax (DrivetrainConstants.kMotorRightRear,MotorType.kBrushless);

  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    motorLeftFront.setIdleMode(IdleMode.kCoast);
    motorLeftRear.setIdleMode(IdleMode.kCoast);
    motorRightFront.setIdleMode(IdleMode.kCoast);
    motorRightRear.setIdleMode(IdleMode.kCoast);
    motorLeftFront.setInverted(true);
    motorLeftRear.setInverted(true);

    motorLeftFront.setSmartCurrentLimit(DrivetrainConstants.kMotorCurrentLimit);
    motorLeftRear.setSmartCurrentLimit(DrivetrainConstants.kMotorCurrentLimit);
    motorRightFront.setSmartCurrentLimit(DrivetrainConstants.kMotorCurrentLimit);
    motorRightRear.setSmartCurrentLimit(DrivetrainConstants.kMotorCurrentLimit);

    motorLeftFront.follow(motorLeftRear);
    motorRightFront.follow(motorRightRear);

    leftEncoder = motorLeftRear.getEncoder();
    rightEncoder = motorRightRear.getEncoder();

    leftEncoder.setPositionConversionFactor(1);
    rightEncoder.setPositionConversionFactor(1);

    
  }
  public void setDriveMotors(double forward, double turn) {
    SmartDashboard.putNumber("drive forward power (%)", forward * 100.0);
    SmartDashboard.putNumber("drive turn power (%)", turn * 100.0);

    /*
     * positive turn = counter clockwise, so the left side goes backwards
     */
    double left = forward - turn;
    double right = forward + turn;

    SmartDashboard.putNumber("drive left power (%)", left * 100.0);
    SmartDashboard.putNumber("drive right power (%)", right * 100.0);

    // see note above in robotInit about commenting these out one by one to set
    // directions.
    motorLeftRear.set(left);
    motorRightRear.set(right);
  }

  public void setTankMotors(double left, double right) {

    SmartDashboard.putNumber("drive left power (%)", left * 100.0);
    SmartDashboard.putNumber("drive right power (%)", right * 100.0);

    // see note above in robotInit about commenting these out one by one to set
    // directions.
    motorLeftRear.set(left);
    motorRightRear.set(right);
  }
  public double getRightEncoder(){
    return rightEncoder.getPosition();
  }
  public double getLeftEncoder(){
    return leftEncoder.getPosition();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Encoder", getLeftEncoder());
    SmartDashboard.putNumber("Right Encoder", getRightEncoder());
  }
}
