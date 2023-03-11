// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;

public class DriveAndTurn extends SequentialCommandGroup {
  
  public DriveAndTurn(double distance, double angle) {
    Drivetrain drive = Drivetrain.getInstance();
    addCommands(
      new ResetOdometry(),
      new InstantCommand(()->drive.breake(true),drive),
      new DriveDistance(distance,drive),
      new WaitCommand(0.5),
      new DriveAngle(angle,drive)
    );
  }
}
