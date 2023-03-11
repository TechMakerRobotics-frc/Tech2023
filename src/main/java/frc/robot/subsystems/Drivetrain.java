// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
  

  // Odometry class for tracking robot pose
  private DifferentialDriveKinematics kinematics;
  private Field2d field = new Field2d();

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
    motorRightFront.setInverted(true);
    motorRightRear.setInverted(true);
    setMaxOutput(false);
    // Busco o objeto encoder de cada modulo e associo
    leftEncoder1 = motorLeftRear.getEncoder();
    rightEncoder1 = motorRightRear.getEncoder();
    leftEncoder2 = motorLeftFront.getEncoder();
    rightEncoder2 = motorRightFront.getEncoder();
    
    // Configuro o fator do encoder para 1 - 1 pulso por volta.
     leftEncoder1.setPositionConversionFactor(DrivetrainConstants.kEncoderDistancePerRotation);
    leftEncoder2.setPositionConversionFactor(DrivetrainConstants.kEncoderDistancePerRotation);
    rightEncoder1.setPositionConversionFactor(DrivetrainConstants.kEncoderDistancePerRotation);
    rightEncoder2.setPositionConversionFactor(DrivetrainConstants.kEncoderDistancePerRotation);
    motorLeftFront.burnFlash();
    motorLeftRear.burnFlash();
    motorRightFront.burnFlash();
    motorRightRear.burnFlash();

    // Seto a posição para no começo
    resetEncoders();

    // Associo a placa de navegação e reseto ela
    m_gyro = new AHRS(DrivetrainConstants.NAVX_PORT);
    resetYaw();
    Pose2d initialPose = new Pose2d(0, 1.5, m_gyro.getRotation2d());
    kinematics =
            new DifferentialDriveKinematics(Units.inchesToMeters(DrivetrainConstants.kTrackwidth));
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(),
        getLeftDistanceMeters(), getRightDistanceMeters(),
        initialPose);

    m_poseEstimator = new DifferentialDrivePoseEstimator(kinematics,
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
public void updatePose() {
  m_odometry.update(m_gyro.getRotation2d(), Units.inchesToMeters(leftEncoder1.getPosition()), Units.inchesToMeters(rightEncoder1.getPosition()));
  field.setRobotPose(m_poseEstimator.getEstimatedPosition());

}

public void resetPose(Pose2d newPose) {
  m_poseEstimator.resetPosition(m_gyro.getRotation2d(), leftEncoder1.getPosition(), rightEncoder1.getPosition(), newPose);
}
  // Periodico só atualiza os dados no Dashboard para informações
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Distancia Esquerda", getLeftDistanceMeters());
    SmartDashboard.putNumber("Distancia Direita", getRightDistanceMeters());
    SmartDashboard.putNumber("Distancia Total", GetAverageEncoderDistance());
    SmartDashboard.putNumber("Giro", getYaw());
    SmartDashboard.putNumber("Roll", getRoll());
    SmartDashboard.putNumber("Angle", getPitch());   
    SmartDashboard.putData("Field",field);

    m_poseEstimator.update(m_gyro.getRotation2d(),
        getLeftDistanceMeters(),
        getRightDistanceMeters());
    updatePose();
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
    SmartDashboard.putNumber("Potencia Frente (%)", forward * 100.0);
    SmartDashboard.putNumber("Potencia Curva (%)", rotation * 100.0);
    m_diffDrive.arcadeDrive(forward, rotation);
    m_diffDrive.feed();

  }

  public void tankDriveVolts(double left, double right) {
    m_leftMotor.setVoltage(left);
    m_rightMotor.setVoltage(right);
    m_diffDrive.feed();
  }
  public void tankDrive(double left, double right) {
    m_leftMotor.set(left);
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
    return getRightEncoder();
  }

  public double getLeftDistanceMeters() {
    return getLeftEncoder();
  }

  public double getRightEncoder() {
    return ((rightEncoder1.getPosition() + rightEncoder2.getPosition()) / 2.0);
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

  public double getRoll(){
    return m_gyro.getRoll();
  }

  public double getPitch(){
    return m_gyro.getPitch();
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
