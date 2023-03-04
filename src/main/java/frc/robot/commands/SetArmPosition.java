// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class SetArmPosition extends CommandBase {
  public enum Position{
    Hold(0),
    Low(8.0),
    Medium(14.0),
    High(26.0),
    Intake(20.0);
    double encoderValue;
    Position(double encoder){
      encoderValue = encoder;
    }
    public double getPosition(){
      return encoderValue;
    }
  }
  Position position;
  Arm arm;
  /** Creates a new SetArmPosition. */
  public SetArmPosition(Arm arm, Position position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.position = position;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(arm.getEncoder()>position.getPosition()){
      arm.setMotorPower(ArmConstants.kPower);
    }
    if(arm.getEncoder()<position.getPosition())
    {
      arm.setMotorPower(ArmConstants.kPower*-1);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(arm.getEncoder())==Math.abs(position.getPosition());
  }
}
