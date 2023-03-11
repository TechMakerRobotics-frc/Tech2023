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
import frc.robot.subsystems.SuperiorIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RetractArm extends SequentialCommandGroup {
  Arm arm = Arm.getInstance();
  SuperiorIntake intake = SuperiorIntake.getInstance();
  public RetractArm() {
    addCommands(
      new InstantCommand(()->arm.setMotorPower(-ArmConstants.kPower),arm),
      new WaitCommand(autonomousConstants.kExtendArmTime),
      new InstantCommand(()->arm.setMotorPower(-ArmConstants.kPowerWait),arm),
      new InstantCommand(()->intake.setLedTeamColor(),intake)
    );
  }
}
