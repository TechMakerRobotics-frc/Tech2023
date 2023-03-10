// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
   * Creates a new DriveAngle. This command will drive your your robot for a desired angle at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param angle The number of meters the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
public class DriveAngle extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_angle;
  private final double m_speed;

  /*
   * Criação do comando capturando os dados
   */
  public DriveAngle(double speed, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = Drivetrain.getInstance();
    m_angle = angle;
    m_speed = speed;

  }

  /*
   * Inicializa as variaveis
   */
  @Override
  public void initialize() {
    m_drive.arcadeDrive(0, 0);
    m_drive.resetYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(0, m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_angle>0){
      return Math.abs(m_drive.getYaw()) >= m_angle;
    }
    return Math.abs(m_drive.getYaw()) <= m_angle;
  }
}
