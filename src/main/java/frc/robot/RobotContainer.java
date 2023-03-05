

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.SetArmPosition.Position;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.AuxiliarIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PDP;
import frc.robot.subsystems.SuperiorIntake;
import frc.robot.subsystems.SuperiorIntake.Element;
import edu.wpi.first.wpilibj.Joystick;
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

  /**
  * Cria os controles para comandar
  * Usamos um controle de Xbox ou manche para navegação
  * E uma central de botões  para os mecanismos
  **/

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final Joystick m_operatorControlller = new Joystick(OperatorConstants.kOperatorControllerPort);
  //Cconfigura os eventos para o intake
  Trigger bIntakeCone = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonIntakeCone);
  Trigger bIntakeCube = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonIntakeCube);
  Trigger bIntakeRelease = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonIntakeRelease);
  Trigger bIntakeArm = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonArmIntake);
  
  //Configura os eventos de  posições do braço
  Trigger bLevelHigh = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonArmLevelHigh);
  Trigger bLevelMedium = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonArmLevelMedium);
  Trigger bLevelLow = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonArmLevelLow);
  Trigger bLevelHold = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonArmLevelHold);

  //Configura os eventos do intake auxiliar
  Trigger bIntakeAuxDown = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonIntakeAuxDown);
  Trigger bIntakeAuxUp = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonIntakeAuxUp);



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
    drive.setDefaultCommand(new RunCommand(()->drive.setDriveMotors(m_driverController.getLeftY(), m_driverController.getRightX(),!arm.getArmZero()), drive));
    //Setando o braço pelos triggers do controle. 
    intakeAux.setDefaultCommand(new RunCommand(()->intakeAux.setMotor((m_driverController.getLeftTriggerAxis()*-0.5)+(m_driverController.getRightTriggerAxis()*0.5)), intakeAux));
    bLevelHigh.onTrue(new SetArmPosition(arm,Position.High));
    bLevelMedium.onTrue(new SetArmPosition(arm,Position.Medium));
    bLevelLow.onTrue(new SetArmPosition(arm,Position.Low));
    bLevelHold.onTrue(new SetArmPosition(arm,Position.Hold));
    bIntakeArm.onTrue(new SetArmPosition(arm,Position.Intake));

    //Seta elementos de intake com comandos do controles de  navegação

    m_driverController.a().onTrue(new InstantCommand(()->intake.intakeElement(Element.Cone),intake));
    m_driverController.b().onTrue(new InstantCommand(()->intake.intakeElement(Element.Cube),intake));
    m_driverController.x().onTrue(new InstantCommand(()->intake.releaseElement(),intake));
    
    bIntakeCone.onTrue(new InstantCommand(()->intake.intakeElement(Element.Cone),intake));
    bIntakeCube.onTrue(new InstantCommand(()->intake.intakeElement(Element.Cube),intake));
    bIntakeRelease.onTrue(new InstantCommand(()->intake.releaseElement(),intake));
    
    bIntakeAuxDown.onTrue(new InstantCommand(()->intakeAux.setIntake(true),intakeAux));
    bIntakeAuxUp.onTrue(new InstantCommand(()->intakeAux.setIntake(false),intakeAux));
    m_driverController.back().onTrue(new InstantCommand(()->arm.resetEncoder(),arm));
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
