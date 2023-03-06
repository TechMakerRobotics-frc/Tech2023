// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
  // Motores de tração
  private CANSparkMax motorLeftFront = new CANSparkMax(DrivetrainConstants.kMotorLeftFront, MotorType.kBrushless);
  private CANSparkMax motorLeftRear = new CANSparkMax(DrivetrainConstants.kMotorLeftRear, MotorType.kBrushless);
  private CANSparkMax motorRightFront = new CANSparkMax(DrivetrainConstants.kMotorRightFront, MotorType.kBrushless);
  private CANSparkMax motorRightRear = new CANSparkMax(DrivetrainConstants.kMotorRightRear, MotorType.kBrushless);
  MotorControllerGroup m_leftMotor = new MotorControllerGroup(motorLeftFront, motorLeftRear);
  MotorControllerGroup m_rightMotor = new MotorControllerGroup(motorRightFront, motorRightRear);
  // Set up the differential drive controller
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

  // Show a field diagram for tracking odometry
  private final Field2d m_field2d = new Field2d();

  private DifferentialDrivePoseEstimator m_poseEstimator;

  // Show a field diagram for tracking Pose estimation
  private final Field2d m_estimatedField2d = new Field2d();

  // Create a slew rate filter to give more control over the speed from the
  // joystick
  private final SlewRateLimiter m_filter = new SlewRateLimiter(0.5);
  private final SlewRateLimiter m_filter_turn = new SlewRateLimiter(0.5);

  // Used to put data onto Shuffleboard
  private ShuffleboardTab driveTab = Shuffleboard.getTab("Drivetrain");

  private GenericEntry m_leftVolts = driveTab.add("Left Volts", 0)
      .withWidget(BuiltInWidgets.kGraph)
      .withPosition(3, 3)
      .getEntry();

  private GenericEntry m_rightVolts = driveTab.add("Right Volts", 0)
      .withWidget(BuiltInWidgets.kGraph)
      .withPosition(3, 3)
      .getEntry();

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
    motorLeftFront.setInverted(true);
    motorLeftRear.setInverted(true);

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
    m_field2d.setRobotPose(initialPose);

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(),
        getLeftDistanceMeters(), getRightDistanceMeters(),
        initialPose);

    m_poseEstimator = new DifferentialDrivePoseEstimator(DrivetrainConstants.kDriveKinematics,
        m_gyro.getRotation2d(),
        getLeftDistanceMeters(), getRightDistanceMeters(),
        initialPose,
        VecBuilder.fill(0.02, 0.02, 0.01),
        VecBuilder.fill(0.1, 0.1, 0.1));

    SmartDashboard.putData("field", m_field2d);
    SmartDashboard.putData("fieldEstimate", m_estimatedField2d);

  }

  // Função principal, movimenta o robo para frente e com curva
  public void setDriveMotors(double forward, double turn) {
    SmartDashboard.putNumber("Potencia Frente (%)", forward * 100.0);
    SmartDashboard.putNumber("Potencia Curva (%)", turn * 100.0);

    /*
     * Volta positiva = Anti horario, esquerda vai para tras
     */
    double left = forward - turn;
    double right = forward + turn;
    setTankMotors(left, right);

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

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly
   * 
   * @param maxOutput The maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_diffDrive.setMaxOutput(maxOutput);
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

  // Movimento em modo de tanque. Util para autonomo e equilibrio
  public void setTankMotors(double left, double right) {

    SmartDashboard.putNumber("Potencia Esquerda (%)", left * 100.0);
    SmartDashboard.putNumber("Potencia Direita (%)", right * 100.0);

    /*
     * Rotina de rampa
     * A cada 20ms o defaultComand passa por essa rotina. Então o incremento
     * da rampa é feito com base em um valor constante e não colocando diretamente
     * o valor dos controles nas saidas. Assim tanto as aceleradas quanto as
     * frenagens
     * são suaves e evitam quedas
     * 
     */
    double leftOutput = getLeftPower();
    double rightOutput = getRightPower();
    if (left > leftOutput) {
      leftOutput = leftOutput + DrivetrainConstants.kRampRate;
      if (leftOutput > left)
        leftOutput = left;
    } else if (left < leftOutput) {
      leftOutput = leftOutput - DrivetrainConstants.kRampRate;
      if (leftOutput < left)
        leftOutput = left;
    }
    if (right > rightOutput) {
      rightOutput = rightOutput + DrivetrainConstants.kRampRate;
      if (rightOutput > right)
        rightOutput = right;
    } else if (right < rightOutput) {
      rightOutput = rightOutput - DrivetrainConstants.kRampRate;
      if (rightOutput < right)
        rightOutput = right;
    }

    // see note above in robotInit about commenting these out one by one to set
    // directions.
    motorLeftRear.set(leftOutput);
    motorLeftFront.set(leftOutput);
    motorRightRear.set(rightOutput);
    motorRightFront.set(rightOutput);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void rateLimitedArcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(m_filter.calculate(xaxisSpeed), m_filter_turn.calculate(zaxisRotate));
  }

  /**
   * The average distance in meters for both wheels
   *
   * @return The average distance in meters for both wheels
   */
  public double getAverageDistanceMeters() {
    return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   * 
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {

    double rightVoltsCalibrated = rightVolts * DrivetrainConstants.rightVoltsGain;

    // Send to Network Tables
    m_leftVolts.setDouble(leftVolts);
    m_rightVolts.setDouble(rightVoltsCalibrated);

    // Apply the voltage to the wheels
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(-rightVoltsCalibrated); // We invert this to maintain +ve = forward
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

  // função de reset dos encoders definindo ponto inicial do robo
  public void resetEncoders() {
    leftEncoder1.setPosition(0);
    leftEncoder2.setPosition(0);
    rightEncoder1.setPosition(0);
    rightEncoder2.setPosition(0);
  }

  /*
   * Funções para pegar os encoders
   * Essas funções retornam as medias dos dois encoders de cada lado
   * Alem disso já faz a conta do numero de pulsos dividido pela razão da caixa de
   * redução
   * e multiplicado pelo perimetro da roda para ter o resultado em metros
   */
  public double getRightDistanceMeters() {
    return ((rightEncoder1.getPosition() + rightEncoder2.getPosition()) / 2 / DrivetrainConstants.kGearboxRatio)
        * DrivetrainConstants.kWheelDiameterMeters * Math.PI;
  }

  public double getLeftDistanceMeters() {
    return ((leftEncoder1.getPosition() + leftEncoder2.getPosition()) / 2 / DrivetrainConstants.kGearboxRatio)
        * DrivetrainConstants.kWheelDiameterMeters * Math.PI;
  }

  public double getRightEncoder() {
    return (rightEncoder1.getPosition() + rightEncoder2.getPosition()) / 2.0;
  }

  public double getLeftEncoder() {
    return (leftEncoder1.getPosition() + leftEncoder2.getPosition()) / 2.0;
  }
 /**
     * Returns the currently estimated pose of the robot.
     * @return The pose
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

  // Periodico só atualiza os dados no Dashboard para informações
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Distancia Esquerda", getLeftEncoder());
    SmartDashboard.putNumber("Distancia Direita", getRightEncoder());
    SmartDashboard.putNumber("Giro", getYaw());
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(),
        getLeftDistanceMeters(),
        getRightDistanceMeters());

    m_poseEstimator.update(m_gyro.getRotation2d(),
        getLeftDistanceMeters(),
        getRightDistanceMeters());

    // Also apply vision measurements. We use 0.3 seconds in the past as an example
    // -- on
    // a real robot, this must be calculated based either on latency or timestamps.
    //m_poseEstimator.addVisionMeasurement(
    //    //getLimelightPose(),
    //    Timer.getFPGATimestamp() - 0.3);
    publishTelemetry();

  }
  public void publishTelemetry() {
      
    m_field2d.setRobotPose(getPose());  
    
    m_estimatedField2d.setRobotPose(m_poseEstimator.getEstimatedPosition());
  
  
    // Offset the pose to start 1.5 meters on the Y axis
    // double yPoseOffset = 1.5;
    // Pose2d currentPose = getPose();
    // Pose2d poseOffset = new Pose2d(currentPose.getX(), 
    //                                currentPose.getY() + yPoseOffset, 
    //                                currentPose.getRotation());
    // // Update the Field2D object (so that we can visualize this in sim)
    // m_field2d.setRobotPose(poseOffset);

    // Updates the the Unscented Kalman Filter using only wheel encoder information.
    // m_estimator.update(m_gyro.getRotation2d(), 
    //                   // getWheelSpeeds(), 
    //                    m_leftEncoder.getDistance(), 
    //                    m_rightEncoder.getDistance());


    // // Offset the pose to start 1.5 meters on the Y axis
    // Pose2d currentEstimatedPose = getEstimatedPose();
    // Pose2d estimatedPoseOffset = new Pose2d(currentEstimatedPose.getX(), 
    //                                         currentEstimatedPose.getY() + yPoseOffset, 
    //                                         currentEstimatedPose.getRotation());

    // // Update the Field2D object (so that we can visualize this in sim)
    // m_estimatedField2d.setRobotPose(estimatedPoseOffset);

    // Display the meters per/second for each wheel and the heading
    
  }
}
