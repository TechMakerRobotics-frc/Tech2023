// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class SetArmPosition extends CommandBase {
  boolean returning = false;
  public enum Position{
    Hold(ArmConstants.kHoldPosition),
    Low(ArmConstants.kLowPosition),
    Medium(ArmConstants.kMediumPosition),
    High(ArmConstants.kHighPosition),
    Intake(ArmConstants.kIntakePosition);
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
    //Testo se o braço está  retornando
    //para que quando passar da posição do encoder, encerrar o comando
    returning = (position==Position.Hold);
    
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Braco retornando", returning);
    SmartDashboard.putNumber("Braco posicao desejada", position.getPosition());
    double error = position.getPosition()-arm.getEncoder();
    double output = error * ArmConstants.kp;
    if(Math.abs(output)>ArmConstants.kPower){
      if(output>0){
        output = ArmConstants.kPower;
      }
      else
        output = -ArmConstants.kPower;
    }
    if(!returning){
      arm.setMotorPower(output);
    }
    else
    {
      arm.setMotorPower(output);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!returning)
      return (arm.getEncoder()>(position.getPosition()));
    return (arm.getEncoder()<(position.getPosition()));
  }
}
