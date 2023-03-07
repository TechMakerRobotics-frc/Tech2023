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
  //Classe do motor do intake superior do  robo
  VictorSPX motor = new VictorSPX(IntakeConstants.kIntakeMotor);
  //Cria uma lista de elementos possiveis. Assim associamos diversas caracteristicas
  // dos elementos para ser usado internamente  na classe
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
  //Mantemos o  ultimo elemento capturado  para saber a direção do descarte
  private Element lastElement = Element.None;
  //Timer para captura
  private double time;

  public SuperiorIntake(Element start) {
    //Inicia com o ultimo elemento zerado
    lastElement = start;
    //Limpa as configurações do motor e seta para ele se  manter estatico e a rampa de aceleração
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configOpenloopRamp(IntakeConstants.kRampRate);
  }
  /*
   * Função de captura  de elemento
   * Conforme o elemento passado para ela, ela ajusta a direção que o motor deve girar
   * para capturar o elemento correto
   * Ainda, ajusta o  timer, para que o  motor só gire pouco tempo, o  suficiente para manter  o  elemento
   */
  public void intakeElement(Element element){
    if(lastElement==Element.None || lastElement==element){
      if(element==Element.Cube)
        motor.set(VictorSPXControlMode.PercentOutput, element.getDirection());
      if(element==Element.Cone)
      motor.set(VictorSPXControlMode.PercentOutput, element.getDirection());
      
      lastElement = element;
      
    }
  }
  public void stopIntake(){
    motor.set(VictorSPXControlMode.PercentOutput, 0);
  }
  //Aqui libera  o elemento, com o sentido contrario a captura. E já coloca o  estado como nenhum
  public void releaseElement(){
    if(lastElement!=Element.None){
      time = Timer.getFPGATimestamp()+1;
      motor.set(VictorSPXControlMode.PercentOutput, lastElement.getDirection()*-1);
      lastElement = Element.None;
    }
  }
  //O periodico atualiza o Dashboard, alem de parar o motor depois do tempo  de cada elemento
  @Override
  public void periodic() {
    SmartDashboard.putString("Elemento atual", lastElement.getName());
    if(Timer.getFPGATimestamp()>time && lastElement==Element.None)
      motor.set(VictorSPXControlMode.PercentOutput, 0);

  }
}
