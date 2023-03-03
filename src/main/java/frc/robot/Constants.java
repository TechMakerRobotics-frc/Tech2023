
package frc.robot;

import edu.wpi.first.wpilibj.SPI;

public final class Constants {
  /*
   * 
   */
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kbuttonintakecolletcube = 9; /*cubo */
    public static final int kbuttonintakecolletcone = 10; /*cone */
    public static final int kbuttonintakerelease= 11; /*soltar */
    public static final int kbuttonintakekelevel1 = 5;/*estaaçao nivel baixa  */
    public static final int kbuttonintakekelevel2 =6;/*estaçao nivel  media */
    public static final int kbuttonintakekelevel3 =7;/*estacao nivel alta */
  }
 

  public static class DrivetrainConstants {
    public static final int kMotorLeftFront = 1;
    public static final int kMotorLeftRear = 2;
    public static final int kMotorRightFront = 3;
    public static final int kMotorRightRear = 4;
    public static final int kMotorCurrentLimit = 28;
    public static final SPI.Port NAVX_PORT = SPI.Port.kMXP;
    public static final double kGearboxRatio = 8.45;
    public static final double kWheelDistance = 6*Math.PI*0.0254;//converte 1 volta da roda em metros

  }
  public static class IntakeConstants{
    public static final int kIntakeMotor = 9;
    public static final int kIntakeCube = 1;
    public static final int kIntakeCone = -1;
    public static final int kIntakeCubeRevert = kIntakeCone;
    public static final int kIntakeConeRevert = kIntakeCube;
    public static final int kIntakeStop = 0;
    public static final double kIntakeTimeCone = 0.5;
    public static final double kIntakeTimeCube = 0.2;
  }
  public static class ArmConstants{
    public static final int kArmLeftMotor = 5;
    public static final int kArmRighrMotor = 6;
    public static final int kGearRatio = 60;
    
  }

}




