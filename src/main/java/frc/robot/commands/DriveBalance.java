// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.autonomousConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveBalance extends CommandBase {
  Drivetrain drive = Drivetrain.getInstance();
  int lastDistancePosition = 0;
  double lastDistance = autonomousConstants.kDistanceToPark[lastDistancePosition];
  boolean parked = false;
  boolean startPositioning = false;
  boolean waiting = false;
  int direction = 1;
  double timeout;
  public DriveBalance() {
    drive.resetEncoders();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastDistancePosition = 0;
    lastDistance = autonomousConstants.kDistanceToPark[lastDistancePosition];
    parked = false;
    startPositioning = false;
    waiting = false;
    direction = 1;
    drive.arcadeDrive(autonomousConstants.kDriveSpeed, 0);
    drive.breake(true);
    drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(parked){
      drive.arcadeDrive(drive.GetAverageEncoderDistance()*-20, 0);
      
    }
    else if(Math.abs(drive.getRoll())>autonomousConstants.kMaxAngle && startPositioning==false){
      startPositioning = true;
      drive.resetEncoders();
      drive.arcadeDrive(autonomousConstants.kDriveSpeed, 0);
    }
    else if(startPositioning){
      
      if(Math.abs(drive.GetAverageEncoderDistance()) >= Math.abs(lastDistance)){
          lastDistancePosition++;
          lastDistance = autonomousConstants.kDistanceToPark[lastDistancePosition];
          direction = (int)(lastDistance/Math.abs(lastDistance));
          double speed = (lastDistancePosition>=1?(autonomousConstants.kDriveSpeedSlow*direction):autonomousConstants.kDriveSpeed);
          drive.arcadeDrive(speed, 0);
          drive.resetEncoders();

      }
      else if(waiting)
      {
        drive.arcadeDrive(0,0);
      }
      else{
        drive.arcadeDrive(autonomousConstants.kDriveSpeed*direction, 0);
      }
      if(lastDistancePosition>=autonomousConstants.kDistanceToPark.length-1)
      {
        parked=true;
        drive.arcadeDrive(0, 0);
        drive.resetEncoders();
        timeout = Timer.getFPGATimestamp()+5;
      }
      SmartDashboard.putNumber("Distancia atual", drive.GetAverageEncoderDistance());
      SmartDashboard.putNumber("Distancia", lastDistance);
      SmartDashboard.putNumber("Sinal", direction);
      
    }
    else if(startPositioning==false){
      drive.arcadeDrive(autonomousConstants.kDriveSpeed, 0);
    }
   

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp()>timeout);
  }
}
