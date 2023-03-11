// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.autonomousConstants;
import frc.robot.subsystems.Arm;

public class ExtendArm extends SequentialCommandGroup {
  Arm arm = Arm.getInstance();
  public ExtendArm() {
    addCommands(
      new InstantCommand(()->arm.setMotorPower(ArmConstants.kPower),arm),
      new WaitCommand(autonomousConstants.kExtendArmTime),
      new InstantCommand(()->arm.setMotorPower(ArmConstants.kPowerWait),arm)
    );
  }
}
