// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drivetrain extends SubsystemBase {
  //Motores de tração
  private CANSparkMax  motorLeftFront = new CANSparkMax(DrivetrainConstants.kMotorLeftFront,MotorType.kBrushless);
  private CANSparkMax  motorLeftRear = new CANSparkMax (DrivetrainConstants.kMotorLeftRear,MotorType.kBrushless);
  private CANSparkMax  motorRightFront = new CANSparkMax (DrivetrainConstants.kMotorRightFront,MotorType.kBrushless);
  private CANSparkMax  motorRightRear = new CANSparkMax (DrivetrainConstants.kMotorRightRear,MotorType.kBrushless);

  //Encoders. São utilizados os enconders internos dos motores
  //Para maior precisão é usada a media dos dois motores
  private RelativeEncoder leftEncoder1, leftEncoder2;
  private RelativeEncoder rightEncoder1, rightEncoder2;

  //Placa de navegação
  private AHRS navX;


  /** Creates a new Drivetrain. */
  public Drivetrain() {
    //Reseto qualquer configuração nos modulos e refaço elas do zero
    motorLeftFront.restoreFactoryDefaults();
    motorLeftRear.restoreFactoryDefaults();
    motorRightFront.restoreFactoryDefaults();
    motorRightRear.restoreFactoryDefaults();
    //Deixo os motores girarem livre para não tombar
    motorLeftFront.setIdleMode(IdleMode.kCoast);
    motorLeftRear.setIdleMode(IdleMode.kCoast);
    motorRightFront.setIdleMode(IdleMode.kCoast);
    motorRightRear.setIdleMode(IdleMode.kCoast);
    //Inverto o sentido da esquerda para rodarem iguais
    motorLeftFront.setInverted(true);
    motorLeftRear.setInverted(true);

    //Configuro a rampa de aceleração dos modulos
    motorLeftFront.setOpenLoopRampRate(DrivetrainConstants.kRampRate);
    motorLeftRear.setOpenLoopRampRate(DrivetrainConstants.kRampRate);
    motorRightFront.setOpenLoopRampRate(DrivetrainConstants.kRampRate);
    motorRightRear.setOpenLoopRampRate(DrivetrainConstants.kRampRate);

    //Limito a corrente dos motores por software para não desligar os fusiveis
    motorLeftFront.setSmartCurrentLimit(DrivetrainConstants.kMotorCurrentLimit);
    motorLeftRear.setSmartCurrentLimit(DrivetrainConstants.kMotorCurrentLimit);
    motorRightFront.setSmartCurrentLimit(DrivetrainConstants.kMotorCurrentLimit);
    motorRightRear.setSmartCurrentLimit(DrivetrainConstants.kMotorCurrentLimit);

    //Busco o objeto encoder de cada modulo e associo 
    leftEncoder1 = motorLeftRear.getEncoder();
    rightEncoder1 = motorRightRear.getEncoder();
    leftEncoder2 = motorLeftFront.getEncoder();
    rightEncoder2 = motorRightFront.getEncoder();
    
    //Configuro o fator do encoder para 1 - 1 pulso por volta. 
    leftEncoder1.setPositionConversionFactor(1);
    rightEncoder1.setPositionConversionFactor(1);
    leftEncoder2.setPositionConversionFactor(1);
    rightEncoder2.setPositionConversionFactor(1);
    
    //Seto a posição para no começo
    resetEncoder();

    //Associo a placa de navegação e reseto ela
    navX = new AHRS(DrivetrainConstants.NAVX_PORT);
    resetYaw();

  }
  //Função principal, movimenta o robo para frente e com curva
  public void setDriveMotors(double forward, double turn) {
    SmartDashboard.putNumber("Potencia Frente (%)", forward * 100.0);
    SmartDashboard.putNumber("Potencia Curva (%)", turn * 100.0);

    /*
     * Volta positiva = Anti horario, esquerda vai para tras
     */
    double left = forward - turn;
    double right = forward + turn;

    SmartDashboard.putNumber("Potencia Esquerda (%)", left * 100.0);
    SmartDashboard.putNumber("Potencia Direita (%)", right * 100.0);

    motorLeftRear.set(left);
    motorLeftFront.set(left);
    motorRightRear.set(right);
    motorRightFront.set(right);
  }
  //Movimento em  modo de  tanque. Util para autonomo e equilibrio
  public void setTankMotors(double left, double right) {

    SmartDashboard.putNumber("Potencia Esquerda (%)", left * 100.0);
    SmartDashboard.putNumber("Potencia Direita (%)", right * 100.0);

    // see note above in robotInit about commenting these out one by one to set
    // directions.
    motorLeftRear.set(left);
    motorLeftFront.set(left);
    motorRightRear.set(right);
    motorRightFront.set(right);
  }
  //função de reset dos encoders definindo ponto inicial do robo
  public void resetEncoder(){
    leftEncoder1.setPosition(0);
    leftEncoder2.setPosition(0);
    rightEncoder1.setPosition(0);
    rightEncoder2.setPosition(0);
  }
  /*
   * Funções para  pegar os encoders
   * Essas funções retornam as medias dos dois encoders de cada lado
   * Alem disso já faz a conta do numero de pulsos dividido pela razão da caixa de redução
   * e multiplicado pelo perimetro da roda para ter o resultado em metros
   */
  public double getRightEncoder(){
    return ((rightEncoder1.getPosition()+rightEncoder2.getPosition())/2/DrivetrainConstants.kGearboxRatio)*DrivetrainConstants.kWheelDistance;
  }
  public double getLeftEncoder(){
    return ((leftEncoder1.getPosition()+leftEncoder2.getPosition())/2/DrivetrainConstants.kGearboxRatio)*DrivetrainConstants.kWheelDistance;
  }

  //Captura o angulo que o robo está apontando
  public double getYaw() {
		return navX.getYaw();
	}
  //Zera a direção do  robo
  public void resetYaw(){
    navX.reset();
  }

  //Periodico só  atualiza os dados no Dashboard para informações
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Distancia Esquerda", getLeftEncoder());
    SmartDashboard.putNumber("Distancia Direita", getRightEncoder());
    SmartDashboard.putNumber("Giro", getYaw());
  }
}
