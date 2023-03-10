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
  double setpoint = 0;
  double errorSum = 0;
  double lastTimestamp = 0;
  double lastError = 0;   
  public DriveBalance() {
    drive.resetEncoders();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.arcadeDrive(0, 0);
    drive.breake(true);
    drive.removeDefaultCommand();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // get sensor position
    double sensorPosition = drive.getRoll();

    double kP = SmartDashboard.getNumber("kP", autonomousConstants.kP);
    double kI = SmartDashboard.getNumber("kI", autonomousConstants.kI);
    double kD = SmartDashboard.getNumber("kD", autonomousConstants.kD);
    // calculations
    double error = setpoint - sensorPosition;
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    if (Math.abs(error) < autonomousConstants.kLimit) {
      errorSum += error * dt;
    }

    double errorRate = (error - lastError) / dt;

    double outputSpeed = kP * error + kI * errorSum + kD * errorRate;

    drive.arcadeDrive(outputSpeed, 0);

    // update last- variables
    lastTimestamp = Timer.getFPGATimestamp();
    lastError = error;

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
