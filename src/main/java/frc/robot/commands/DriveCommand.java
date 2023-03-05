// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

public class DriveCommand extends CommandBase {
  /** Creates a new DriveCommand. */
  public DriveCommand(double forward, double turn, Drivetrain drive, Arm arm) {
      addRequirements(drive);
      double left = forward - turn;
      double right = forward + turn;
      double leftOutput = drive.getLeftPower();
      double rightOutput = drive.getRightPower();
      if(left>leftOutput){
        leftOutput = leftOutput+DrivetrainConstants.kRampRate;
        if(leftOutput>left)
          leftOutput = left;
        if(arm.getArmZero() && leftOutput>DrivetrainConstants.kMaxSpeedArmExtended)
         leftOutput = DrivetrainConstants.kMaxSpeedArmExtended;
      }
      else if(left<leftOutput){
        leftOutput = leftOutput-DrivetrainConstants.kRampRate;
        if(leftOutput<left)
          leftOutput = left;
        if(arm.getArmZero() && leftOutput>DrivetrainConstants.kMaxSpeedArmExtended)
         leftOutput = DrivetrainConstants.kMaxSpeedArmExtended;
      }
      if(right>rightOutput){
        rightOutput = rightOutput+DrivetrainConstants.kRampRate;
        if(rightOutput>right)
        rightOutput = right;
        if(arm.getArmZero() && rightOutput>DrivetrainConstants.kMaxSpeedArmExtended)
        rightOutput = DrivetrainConstants.kMaxSpeedArmExtended;
      }
      else if(right<rightOutput){
        rightOutput = rightOutput-DrivetrainConstants.kRampRate;
        if(rightOutput<right)
        rightOutput = right;
        if(arm.getArmZero() && rightOutput>DrivetrainConstants.kMaxSpeedArmExtended)
        rightOutput = DrivetrainConstants.kMaxSpeedArmExtended;
      }
  
      drive.setTankMotors(leftOutput, rightOutput);
      
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
