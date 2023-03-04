

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SuperiorIntake;
import frc.robot.subsystems.SuperiorIntake.Element;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Essa classe junta todos os elementos do  robo e  gerencia os funcionamento geral,
 * associando subsistemas e comandos
 */
public class RobotContainer {
  // Cria um objeto de cada subsistema
  private final Drivetrain drive = new Drivetrain();
  private final Arm arm  = new Arm();
  private final SuperiorIntake intake;

  /**
  * Cria os controles para comandar
  * Usamos um controle de Xbox ou manche para navegação
  * E uma central de botões  para os mecanismos
  **/

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  //private final Joystick m_operatorControlller = new Joystick(OperatorConstants.kOperatorControllerPort);
 
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
    drive.setDefaultCommand(new RunCommand(()->drive.setDriveMotors(m_driverController.getLeftY(), m_driverController.getRightX()), drive));
    //Setando o braço pelos triggers do controle. 
    //TODO - Alterar para posições fixas com  o comando de botoes
    arm.setDefaultCommand(new RunCommand(()->arm.setMotorPower((m_driverController.getLeftTriggerAxis()*-0.5)+(m_driverController.getRightTriggerAxis()*0.5)), arm));
    
    //Seta elementos de intake com comandos do controles de  navegação
    //#TODO Configurar para o comando  de botões
    m_driverController.a().onTrue(new InstantCommand(()->intake.intakeElement(Element.Cone),intake));
    m_driverController.b().onTrue(new InstantCommand(()->intake.intakeElement(Element.Cube),intake));
    m_driverController.x().onTrue(new InstantCommand(()->intake.releaseElement(),intake));


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
