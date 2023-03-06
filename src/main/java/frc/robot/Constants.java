
package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public final class Constants {
  /*
   * 
   */
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kButtonIntakeCube = 9; /* cubo */
    public static final int kButtonIntakeCone = 10; /* cone */
    public static final int kButtonIntakeRelease = 11; /* soltar */
    public static final int kButtonArmIntake = 12; // Braço na posição de intake
    public static final int kButtonArmLevelHigh = 5;/* estaaçao nivel alta */
    public static final int kButtonArmLevelMedium = 6;/* estaçao nivel media */
    public static final int kButtonArmLevelLow = 7;/* estacao nivel baixa */
    public static final int kButtonArmLevelHold = 8; // Braço recolhido
    public static final int kButtonIntakeAuxDown = 3; // Baixa intake Auxiliar
    public static final int kButtonIntakeAuxUp = 4; // Sobe intake auxiliar

  }

  public static class DrivetrainConstants {
    public static final int kMotorLeftFront = 1;
    public static final int kMotorLeftRear = 2;
    public static final int kMotorRightFront = 3;
    public static final int kMotorRightRear = 4;
    public static final int kMotorCurrentLimit = 40;
    public static final SPI.Port NAVX_PORT = SPI.Port.kMXP;
    public static final double kGearboxRatio = 8.45;
    public static final double kWheelDiameterMeters = 6 * 0.0254;// converte 1 volta da roda em metros
    public static final double kRampRate = 0.025;
    public static final double kMaxSpeedArmExtended = 0.4;
    public static final double kp = 0.1;
    public static final double ki = 0.0;
    public static final double kd = 0.01;
    public static final int kCountsPerRevolution = 42;
    // -------- Physical Constants -----------------
    public static final double kMetersPerDegree = Math.PI * kWheelDiameterMeters / 360;
    public static final double kTrackwidthMeters = 0.43;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);

    // Calibration for the right wheel voltage because it's much slower
    // than the left wheel on this robot.
    // public static final double rightVoltsGain = 1.094; // Romi
    public static final double rightVoltsGain = 1.0; // Romi-2

    // -------- Dynamical constants --------------------

    // Max speed and acceleration of the robot
    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAccelMetersPerSecondSquared = 0.5;

    // The linear inertia gain, volts
    public static final double ksVolts = 0.461;
    // The linear velocity gain, volts per (meter per second)
    // Increase this if you drive short
    public static final double kvVoltSecondsPerMeter = 6.93;
    // public static final double kvVoltSecondsPerMeter = 9.7;
    // The linear acceleration gain, volts per (meter per second squared).
    public static final double kaVoltSecondsSquaredPerMeter = 0.0737;

    // Setup constraints for feedforward and kinematics
    public static final SimpleMotorFeedforward kFeedForward = new SimpleMotorFeedforward(ksVolts,
        kvVoltSecondsPerMeter,
        kaVoltSecondsSquaredPerMeter);

    // -------- Trajectory constants --------------------

    // Voltage constraints
    public static final DifferentialDriveVoltageConstraint kAutoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        kFeedForward,
        kDriveKinematics,
        10);

    // Setup trajectory constraints
    public static final TrajectoryConfig kTrajectoryConfig = new TrajectoryConfig(kMaxSpeedMetersPerSecond,
        kMaxAccelMetersPerSecondSquared)
        .setKinematics(kDriveKinematics)
        .addConstraint(kAutoVoltageConstraint);

    // -------- PID constants --------------------
    public static final double kPDriveVel = 0.125;
    public static final double kIDriveVel = 0;
    public static final double kDDriveVel = 0;

    public static final double kPDriveVelLeft = 0.125;
    public static final double kPDriveVelRight = 0.125;
  }

  public static class IntakeConstants {
    public static final int kIntakeMotor = 3;
    public static final double kIntakeCube = 1;
    public static final double kIntakeCone = -1;
    public static final double kIntakeCubeRevert = kIntakeCone;
    public static final double kIntakeConeRevert = kIntakeCube;
    public static final int kIntakeStop = 0;
    public static final double kIntakeTimeCone = 0.5;
    public static final double kIntakeTimeCube = 0.5;
    public static final double kRampRate = 0.05;

  }

  public static class ArmConstants {
    public static final int kArmLeftMotor = 5;
    public static final int kArmRighrMotor = 6;
    public static final int kGearRatio = 60;
    public static final int kReturnTicks = 7;
    public static final int kMaxForwardTicks = 27;
    public static final double kRampRate = 0.3;
    public static final double kPower = 0.3;
    public static final double kZeroPosition = 100;
    public static final int kHoldPosition = 0;
    public static final int kLowPosition = 360;
    public static final int kMediumPosition = 880;
    public static final int kHighPosition = 1600;
    public static final int kIntakePosition = 1300;
    public static final double kp = 0.1;
    public static final double ki = 0.0;
    public static final double kd = 0.01;
  }

  public static class AuxiliarIntakeConstants {
    public static final int kIntakeMotor = 2;
    public static final int kEncoderA = 0;
    public static final int kEncoderB = 1;
    public static final int kPulsesDown = 2048;
    public static final double kMotorPower = 0.7;
  }

  public static class PDPConstants {
    public static final int kID = 1;
    public static final ModuleType kModule = ModuleType.kCTRE;
    public static final double kMinimumVoltage = 10.5;
  }

}
