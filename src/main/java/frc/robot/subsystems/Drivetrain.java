// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance;

  // Motores de tração
  private CANSparkMax motorLeftFront = new CANSparkMax(DrivetrainConstants.kMotorLeftFront, MotorType.kBrushless);
  private CANSparkMax motorLeftRear = new CANSparkMax(DrivetrainConstants.kMotorLeftRear, MotorType.kBrushless);
  private CANSparkMax motorRightFront = new CANSparkMax(DrivetrainConstants.kMotorRightFront, MotorType.kBrushless);
  private CANSparkMax motorRightRear = new CANSparkMax(DrivetrainConstants.kMotorRightRear, MotorType.kBrushless);
  MotorControllerGroup m_leftMotor = new MotorControllerGroup(motorLeftFront, motorLeftRear);
  MotorControllerGroup m_rightMotor = new MotorControllerGroup(motorRightFront, motorRightRear);
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // Encoders. São utilizados os enconders internos dos motores
  // Para maior precisão é usada a media dos dois motores
  private RelativeEncoder leftEncoder1, leftEncoder2;
  private RelativeEncoder rightEncoder1, rightEncoder2;

  // Placa de navegação
  private AHRS m_gyro;
  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  // Odometry class for tracking robot pose

  private final DifferentialDriveOdometry m_odometry;

  private DifferentialDrivePoseEstimator m_poseEstimator;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Reseto qualquer configuração nos modulos e refaço elas do zero
    motorLeftFront.restoreFactoryDefaults();
    motorLeftRear.restoreFactoryDefaults();
    motorRightFront.restoreFactoryDefaults();
    motorRightRear.restoreFactoryDefaults();
    // Deixo os motores girarem livre para não tombar
    motorLeftFront.setIdleMode(IdleMode.kCoast);
    motorLeftRear.setIdleMode(IdleMode.kCoast);
    motorRightFront.setIdleMode(IdleMode.kCoast);
    motorRightRear.setIdleMode(IdleMode.kCoast);
    // Inverto o sentido da esquerda para rodarem iguais
    //motorLeftFront.setInverted(true);
    //motorLeftRear.setInverted(true);
    setMaxOutput(false);
    // Busco o objeto encoder de cada modulo e associo
    leftEncoder1 = motorLeftRear.getEncoder();
    rightEncoder1 = motorRightRear.getEncoder();
    leftEncoder2 = motorLeftFront.getEncoder();
    rightEncoder2 = motorRightFront.getEncoder();
    
    // Configuro o fator do encoder para 1 - 1 pulso por volta.
    leftEncoder1.setPositionConversionFactor(1);
    rightEncoder1.setPositionConversionFactor(1);
    leftEncoder2.setPositionConversionFactor(1);
    rightEncoder2.setPositionConversionFactor(1);

    // Seto a posição para no começo
    resetEncoders();

    // Associo a placa de navegação e reseto ela
    m_gyro = new AHRS(DrivetrainConstants.NAVX_PORT);
    resetYaw();
    Pose2d initialPose = new Pose2d(0, 1.5, m_gyro.getRotation2d());

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(),
        getLeftDistanceMeters(), getRightDistanceMeters(),
        initialPose);

    m_poseEstimator = new DifferentialDrivePoseEstimator(DrivetrainConstants.kDriveKinematics,
        m_gyro.getRotation2d(),
        getLeftDistanceMeters(), getRightDistanceMeters(),
        initialPose,
        VecBuilder.fill(0.02, 0.02, 0.01),
        VecBuilder.fill(0.1, 0.1, 0.1));

  }
  public static Drivetrain getInstance() {
    if (instance == null) {
        instance = new Drivetrain();
    }
    return instance;
}
  // Periodico só atualiza os dados no Dashboard para informações
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Distancia Esquerda", getLeftDistanceMeters());
    SmartDashboard.putNumber("Distancia Direita", getRightDistanceMeters());
    SmartDashboard.putNumber("Giro", getYaw());
    SmartDashboard.putData("Acelerometro", m_accelerometer);
    SmartDashboard.putNumber("Motor esquerdo1", motorLeftFront.get());
    SmartDashboard.putNumber("Motor esquerdo2", motorLeftRear.get());
    SmartDashboard.putNumber("Motor direito1", motorRightFront.get());
    SmartDashboard.putNumber("Motor direito2", motorRightRear.get());
    
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(),
        getLeftDistanceMeters(),
        getRightDistanceMeters());

    m_poseEstimator.update(m_gyro.getRotation2d(),
        getLeftDistanceMeters(),
        getRightDistanceMeters());

  }

  /*
   * Funcões de movimentação
   */

  // Função principal, movimenta o robo para frente e com curva
  public void setDriveMotors(double forward, double turn) {
    SmartDashboard.putNumber("Potencia Frente (%)", forward * 100.0);
    SmartDashboard.putNumber("Potencia Curva (%)", turn * 100.0);

    /*
     * Volta positiva = Anti horario, esquerda vai para tras
     */
    double left = forward - turn;
    double right = forward + turn;
    tankDrive(left, right);

  }

  /**
   * Configura o drivetrain para limitar a velocidade
   * importante quando o braço está extendido
   */
  public void setMaxOutput(boolean set) {
    if(set)
      m_diffDrive.setMaxOutput(DrivetrainConstants.kMaxSpeedArmExtended);
    else
      m_diffDrive.setMaxOutput(1.0);
  }
  public void breake(boolean set){
    if(set){
      motorLeftFront.setIdleMode(IdleMode.kBrake);
      motorLeftRear.setIdleMode(IdleMode.kBrake);
      motorRightFront.setIdleMode(IdleMode.kBrake);
      motorRightRear.setIdleMode(IdleMode.kBrake);
    }
    else{
      motorLeftFront.setIdleMode(IdleMode.kCoast);
      motorLeftRear.setIdleMode(IdleMode.kCoast);
      motorRightFront.setIdleMode(IdleMode.kCoast);
      motorRightRear.setIdleMode(IdleMode.kCoast);
    }
  }
  public void arcadeDrive(double forward, double rotation) {
    m_diffDrive.arcadeDrive(forward, rotation);

  }

  public void tankDriveVolts(double left, double right) {
    m_leftMotor.setVoltage(left);
    m_rightMotor.setVoltage(right);
    m_diffDrive.feed();
  }
  public void tankDrive(double left, double right) {
    m_leftMotor.set(-left);
    m_rightMotor.set(right);
    m_diffDrive.feed();
    
  }
  public void stopDrivetrain() {
    tankDriveVolts(0, 0);
  }

  public double getLeftPower() {
    return (motorLeftFront.get() + motorLeftRear.get()) / 2;
  }

  public double getRightPower() {
    return (motorRightFront.get() + motorRightRear.get()) / 2;
  }

  /*
   * Funções para pegar os encoders
   * Essas funções retornam as medias dos dois encoders de cada lado
   * Alem disso já faz a conta do numero de pulsos dividido pela razão da caixa de
   * redução
   * e multiplicado pelo perimetro da roda para ter o resultado em metros
   */
  public double GetAverageEncoderDistance() {
    return ((getRightDistanceMeters() + getLeftDistanceMeters()) / 2.0);
  }

  public double getRightDistanceMeters() {
    return getRightEncoder()*DrivetrainConstants.kEncoderDistancePerPulse;
  }

  public double getLeftDistanceMeters() {
    return getLeftEncoder()*DrivetrainConstants.kEncoderDistancePerPulse;
  }

  public double getRightEncoder() {
    return (rightEncoder1.getPosition() + rightEncoder2.getPosition()) / 2.0;
  }

  public double getLeftEncoder() {
    return (leftEncoder1.getPosition() + leftEncoder2.getPosition()) / 2.0;
  }
 // função de reset dos encoders definindo ponto inicial do robo
 public void resetEncoders() {
  leftEncoder1.setPosition(0);
  leftEncoder2.setPosition(0);
  rightEncoder1.setPosition(0);
  rightEncoder2.setPosition(0);
}
  /**
   * Funções de telemetria
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  // Captura o angulo que o robo está apontando
  public double getYaw() {
    return m_gyro.getYaw();
  }

  // Zera a direção do robo
  public void resetYaw() {
    m_gyro.reset();
  }

   /**
   * Zeroes the heading of the robot
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  /**
   * Resets the odometry to the specified pose
   * 
   * @param pose The pose to which to set the odometry
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    zeroHeading();
    m_odometry.resetPosition(m_gyro.getRotation2d(),
        getLeftDistanceMeters(), getRightDistanceMeters(),
        pose);
  }

}
