// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private static Limelight instance;
  /** Creates a new Limelight. */
  public Limelight() {}

  public static Limelight getInstance(){
    if(instance==null){
      instance = new Limelight();
    }
    return instance;
  }
  public void ledOn(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

  }

  public void ledOff(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
