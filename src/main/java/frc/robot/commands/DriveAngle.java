// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

/**
   * Creates a new DriveAngle. This command will drive your your robot for a desired angle at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param angle The number of meters the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
public class DriveAngle extends PIDCommand {
  
  /*
   * Criação do comando capturando os dados
   */
  public DriveAngle( double angle, Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(
        new PIDController(DrivetrainConstants.kTurnP, DrivetrainConstants.kTurnI, DrivetrainConstants.kTurnD),
        // Close loop on heading
        drive::getYaw,
        // Set reference to target
        angle,
        // Pipe output to turn robot
        output -> drive.arcadeDrive(0, output),
        // Require the drive
        drive);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DrivetrainConstants.kTurnToleranceDeg, DrivetrainConstants.kTurnRateToleranceDegPerS);

  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
