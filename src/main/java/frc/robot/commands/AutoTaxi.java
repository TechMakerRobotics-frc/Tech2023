// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.autonomousConstants;
import frc.robot.subsystems.SuperiorIntake.Element;

public class AutoTaxi extends SequentialCommandGroup {
  public AutoTaxi(Element start) {
    addCommands(
      new ExtendArm(),
      new ReleaseElementIntake(start),
      new RetractArm(),
      new DriveDistance(autonomousConstants.kDriveSpeed, 4)
    );
  }
}
