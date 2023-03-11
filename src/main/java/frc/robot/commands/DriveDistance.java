// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class DriveDistance extends PIDCommand {

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a
   * desired distance at
   * a desired speed.
   *
   * @param distance The number of meters the robot will drive
   * @param drive  The drivetrain subsystem on which this command will run
   */
  public DriveDistance(double distance, Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(
        new PIDController(DrivetrainConstants.kDriveD, DrivetrainConstants.kDriveI, DrivetrainConstants.kDriveD),
        // Close loop on heading
        drive::GetAverageEncoderDistance,
        // Set reference to target
        distance,
        // Pipe output to turn robot
        output -> drive.arcadeDrive(output,0),
        // Require the drive
        drive);

    // Set the controller tolerance - the delta tolerance ensures the robot is
    // stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DrivetrainConstants.kDriveTolerance, DrivetrainConstants.kDriveRateToleranceMPerS);
  }
 // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}