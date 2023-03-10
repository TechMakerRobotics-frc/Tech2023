// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PDPConstants;

public class PDP extends SubsystemBase {
  private static PDP instance;
  /** Creates a new PDP. */
  PowerDistribution m_pdp = new PowerDistribution(PDPConstants.kID, PDPConstants.kModule);

  public PDP() {
  }
  public boolean getLowVoltage(){
    return (m_pdp.getVoltage()<PDPConstants.kMinimumVoltage);
  }
  public static PDP getInstance() {
    if (instance == null) {
        instance = new PDP();
    }
    return instance;
}

  @Override
  public void periodic() {
    SmartDashboard.putData(m_pdp);
    SmartDashboard.putNumber("PDP Tensao", m_pdp.getVoltage());
    SmartDashboard.putNumber("PDP Corrente", m_pdp.getTotalCurrent());





  }
}
