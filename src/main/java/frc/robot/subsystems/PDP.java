// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PDP extends SubsystemBase {
  /** Creates a new PDP. */
  PowerDistribution m_pdp = new PowerDistribution(1, ModuleType.kCTRE);

  public PDP() {
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("PDP Tensao", m_pdp.getVoltage());
    SmartDashboard.putNumber("PDP Temperatura", m_pdp.getTemperature());
    SmartDashboard.putNumber("PDP Total Corrent", m_pdp.getTotalCurrent());
    SmartDashboard.putNumber("PDP Corrente Esquerda 1", m_pdp.getCurrent(0));
    SmartDashboard.putNumber("PDP Corrente Esquerda 2", m_pdp.getCurrent(1));
    SmartDashboard.putNumber("PDP Corrente Direita 1", m_pdp.getCurrent(14));
    SmartDashboard.putNumber("PDP Corrente Direita 2", m_pdp.getCurrent(15));




  }
}
