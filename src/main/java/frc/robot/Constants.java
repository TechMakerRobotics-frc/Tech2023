
package frc.robot;

import edu.wpi.first.wpilibj.SPI;

public final class Constants {
  /*
   * 
   */
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kButtonIntakeCube = 9; /*cubo */
    public static final int kButtonIntakeCone = 10; /*cone */
    public static final int kButtonIntakeRelease= 11; /*soltar */
    public static final int kButtonArmIntake = 12; //Braço na posição de intake
    public static final int kButtonArmLevelHigh = 5;/*estaaçao nivel alta  */
    public static final int kButtonArmLevelMedium =6;/*estaçao nivel  media */
    public static final int kButtonArmLevelLow =7;/*estacao nivel baixa */
    public static final int kButtonArmLevelHold = 8; //Braço recolhido
    public static final int kButtonIntakeAuxDown = 3; //Baixa intake Auxiliar
    public static final int kButtonIntakeAuxUp = 4; //Sobe intake auxiliar


  }
 

  public static class DrivetrainConstants {
    public static final int kMotorLeftFront = 1;
    public static final int kMotorLeftRear = 2;
    public static final int kMotorRightFront = 3;
    public static final int kMotorRightRear = 4;
    public static final int kMotorCurrentLimit = 40;
    public static final SPI.Port NAVX_PORT = SPI.Port.kMXP;
    public static final double kGearboxRatio = 8.45;
    public static final double kWheelDistance = 6*Math.PI*0.0254;//converte 1 volta da roda em metros
    public static final double kRampRate = 0.3;

  }
  public static class IntakeConstants{
    public static final int kIntakeMotor = 3;
    public static final double kIntakeCube = 0.5;
    public static final double kIntakeCone = -0.5;
    public static final double kIntakeCubeRevert = kIntakeCone;
    public static final double kIntakeConeRevert = kIntakeCube;
    public static final int kIntakeStop = 0;
    public static final double kIntakeTimeCone = 0.5;
    public static final double kIntakeTimeCube = 0.2;
    public static final double kRampRate = 0.3;

  }
  public static class ArmConstants{
    public static final int kArmLeftMotor = 5;
    public static final int kArmRighrMotor = 6;
    public static final int kGearRatio = 60;
    public static final int kReturnTicks = 7;
    public static final int kMaxForwardTicks = 27;
    public static final double kRampRate = 0.3;

    
  }

}




