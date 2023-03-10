

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AuxiliarIntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.AuxiliarIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PDP;
import frc.robot.subsystems.SuperiorIntake;
import frc.robot.subsystems.SuperiorIntake.Element;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Essa classe junta todos os elementos do  robo e  gerencia os funcionamento geral,
 * associando subsistemas e comandos
 */
public class RobotContainer {
  // Cria um objeto de cada subsistema
  private final Drivetrain drive = new Drivetrain();
  private final Arm arm  = new Arm();
  private final SuperiorIntake intake;
  private final PDP pdp = new PDP();
  private final AuxiliarIntake intakeAux = new AuxiliarIntake();
  private final Limelight limeLight = new Limelight();
  /**
  * Cria os controles para comandar
  * Usamos um controle de Xbox ou manche para navegação
  * E uma central de botões  para os mecanismos
  **/

  //private final CommandXboxController m_driverController =      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);
  private final Joystick m_operatorControlller = new Joystick(OperatorConstants.kOperatorControllerPort);
  //Cconfigura os eventos para o intake
  Trigger bIntakeCone = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonIntakeCone);
  Trigger bIntakeCube = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonIntakeCube);
  Trigger bIntakeRelease = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonIntakeRelease);
  
  //Configura os eventos de  posições do braço
  Trigger bLevelHigh = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonArmHigh);
  Trigger bLevelLow = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonArmLow);
  Trigger bLevelKeepHigh = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonArmKeepHigh);
  Trigger bLevelKeepLow = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonArmKeepLow);

  Trigger tLowBatt = new Trigger(pdp::getLowVoltage);
  //Configura os eventos do intake auxiliar
  Trigger bIntakeAuxDown = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonIntakeAuxDown);
  Trigger bIntakeAuxUp = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonIntakeAuxUp);
  Trigger bBreake = new JoystickButton(m_operatorControlller, OperatorConstants.kButtonBreake);

  Trigger bLightOn = new JoystickButton(m_driverController,2);
  Trigger bLightOff = new JoystickButton(m_driverController,1);

  /** 
   * Dentro do container é que são guardados organizados os elementos do robo.
   * Sao configurados os sistemas, associados os comandos aos botões, etc
   * */
  public RobotContainer() {
    intake = new SuperiorIntake(Element.None);
    // A função de Bindings que associa os eventos dos controles aos comandos
    configureBindings();
  }

  /**
   * Aqui fazemos toda a configuração que associa eventos a comandos
   * Principais eventos são os comandos dos controles
   */
  private void configureBindings() {
    //Seta a navegação padrão pelo  controle
    //drive.setDefaultCommand(new RunCommand(()->drive.setDriveMotors((m_driverController.getLeftTriggerAxis())+(m_driverController.getRightTriggerAxis()*-1), 
    //                          m_driverController.getRightX()*0.5), drive));
    drive.setDefaultCommand(new RunCommand(()->drive.setDriveMotors((m_driverController.getRawAxis(1)*((m_driverController.getRawAxis(3)+1)/2)), 
    m_driverController.getRawAxis(2)*((m_driverController.getRawAxis(3)+1)/2)), drive));


    bLightOff.onTrue(new InstantCommand(()->limeLight.ledOn() ,limeLight));
    bLightOn.onTrue(new InstantCommand(()->limeLight.ledOff() ,limeLight));

    //Setando o braço pelos triggers do controle. 
    

    bLevelHigh.onTrue(new InstantCommand(()->arm.setMotorPower(ArmConstants.kPower),arm))
              .onFalse(new InstantCommand(()->arm.setMotorPower(0),arm));
    bLevelLow.onTrue(new InstantCommand(()->arm.setMotorPower(-ArmConstants.kPower),arm))
              .onFalse(new InstantCommand(()->arm.setMotorPower(0),arm));
    bLevelKeepHigh.onTrue(new InstantCommand(()->arm.setMotorPower(ArmConstants.kPowerWait),arm));
    bLevelKeepLow.onTrue(new InstantCommand(()->arm.setMotorPower(-ArmConstants.kPowerWait),arm));
    //Seta elementos de intake com comandos do controles de  navegação

    bBreake.onTrue(new InstantCommand(()->drive.breake(true),drive))
          .onFalse(new InstantCommand(()->drive.breake(false),drive));
    
    bIntakeCone.onTrue(new InstantCommand(()->intake.intakeElement(Element.Cone),intake))
                .onFalse(new InstantCommand(()->intake.stopIntake(),intake));
    bIntakeCube.onTrue(new InstantCommand(()->intake.intakeElement(Element.Cube),intake))
                .onFalse(new InstantCommand(()->intake.stopIntake(),intake));
    bIntakeRelease.onTrue(new InstantCommand(()->intake.releaseElement(),intake));
    
    bIntakeAuxDown.onTrue(new InstantCommand(()->intakeAux.setIntake(AuxiliarIntakeConstants.kMotorPower),intakeAux))
                  .onFalse(new InstantCommand(()->intakeAux.setIntake(0),intakeAux));
    bIntakeAuxUp.onTrue(new InstantCommand(()->intakeAux.setIntake(-AuxiliarIntakeConstants.kMotorPower),intakeAux))
                .onFalse(new InstantCommand(()->intakeAux.setIntake(0),intakeAux));

    tLowBatt.onTrue(new InstantCommand(()->SmartDashboard.putString("ALERTA BATERIA", "BATERIA BAIXA")))
            .onFalse(new InstantCommand(()->SmartDashboard.putString("ALERTA BATERIA", "BATERIA OK")));

    
  }

  

  /**
   * Aqui retornamos o comando autonomo  que vamos usar. Caso só tenhamos um, retorna esse
   * Caso  tenhamos mais de um, captura no Dashboard qual usar
   * Seta tambem o intake para saber qual elemento inicia no autonomo
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
