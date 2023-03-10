
package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
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
    public static final int kButtonArmHigh = 5;/* estaaçao nivel alta */
    public static final int kButtonArmLow = 6;/* estaçao nivel media */
    public static final int kButtonArmKeepHigh = 1;/* estacao nivel baixa */
    public static final int kButtonArmKeepLow = 2; // Braço recolhido
    public static final int kButtonIntakeAuxDown = 3; // Baixa intake Auxiliar
    public static final int kButtonIntakeAuxUp = 4; // Sobe intake auxiliar
    public static final int kButtonBreake = 12;

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
    public static final double kTrackwidth = 0.445;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidth);
    
    public static final int kEncoderCPR = 42;
    public static final double kWheelDiameterInches = 6;
    public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters*Math.PI)/kEncoderCPR;
    
    public static final double ks = 1.06;
    public static final double kv = 2.49;
    public static final double ka = 0.0375;
    
    public static final double kPDriveVel = 0.1;
    public static final double kIDriveVel = 0.0;
    public static final double kDDriveVel = 0.0;
    
    public static final double kMaxSpeed = 0.5;
    public static final double kMaxAcceleration = 1;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public static class IntakeConstants {
    public static final int kIntakeMotor = 3;
    public static final double kIntakeCube = 1;
    public static final double kIntakeCone = -1;
    public static final double kIntakeCubeRevert = kIntakeCone;
    public static final double kIntakeConeRevert = kIntakeCube;
    public static final int kIntakeStop = 0;
    public static final double kIntakeTimeCone = 0.75;
    public static final double kIntakeTimeCube = 0.75;
    public static final double kRampRate = 0.05;
    public static final int kPWMChannelRed = 0;
    public static final int kPWMChannelGreen = 1;
    public static final int kPWMChannelBlue = 2;

  }

  public static class ArmConstants {
    public static final int kArmLeftMotor = 5;
    public static final int kArmRighrMotor = 6;
    public static final int kGearRatio = 60;
    public static final int kReturnTicks = 7;
    public static final int kMaxForwardTicks = 27;
    public static final double kRampRate = 0.3;
    public static final double kPower = 0.2;
    public static final double kPowerWait = 0.06;
    public static final double kp = 0.1;
    public static final double ki = 0.0;
    public static final double kd = 0.01;
  }

  public static class AuxiliarIntakeConstants {
    public static final int kIntakeMotor = 2;
    public static final double kMotorPower = 1;
  }

  public static class PDPConstants {
    public static final int kID = 1;
    public static final ModuleType kModule = ModuleType.kCTRE;
    public static final double kMinimumVoltage = 10.5;
  }

}
