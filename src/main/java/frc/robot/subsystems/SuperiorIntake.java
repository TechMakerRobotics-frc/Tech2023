// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class SuperiorIntake extends SubsystemBase {
  VictorSPX motor = new VictorSPX(IntakeConstants.kIntakeMotor);
  public enum Element{
    Cone(IntakeConstants.kIntakeCone,"Cone"),
    None(IntakeConstants.kIntakeStop,"Nada"),
    Cube(IntakeConstants.kIntakeCube,"Cubo");
    private double direction;
    private String name;

    Element(double direction, String name) {
        this.direction = direction;
        this.name = name;
    }

    public double getDirection() {
        return direction;
    }
    public String getName(){
      return name;
    }
  }
  private Element lastElement = Element.None;
  private double time;
  public SuperiorIntake() {
    lastElement = Element.None;
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configOpenloopRamp(IntakeConstants.kRampRate);
  }

  public void intakeElement(Element element){
    double timeout = 0;
    if(lastElement==Element.None){
      if(element==Element.Cube)
        timeout = IntakeConstants.kIntakeTimeCube;
      if(element==Element.Cone)
        timeout = IntakeConstants.kIntakeTimeCone;
      
      lastElement = element;

      time = Timer.getFPGATimestamp()+timeout;
      motor.set(VictorSPXControlMode.PercentOutput, element.getDirection());
    }
  }
  public void releaseElement(){
    if(lastElement!=Element.None){
      time = Timer.getFPGATimestamp()+1;
      motor.set(VictorSPXControlMode.PercentOutput, lastElement.getDirection()*-1);
      lastElement = Element.None;
    }
  }
  @Override
  public void periodic() {
    SmartDashboard.putString("Elemento atual", lastElement.getName());
    if(Timer.getFPGATimestamp()>time)
      motor.set(VictorSPXControlMode.PercentOutput, 0);

  }
}
