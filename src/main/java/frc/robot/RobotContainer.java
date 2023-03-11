

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AuxiliarIntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveAndTurn;
import frc.robot.commands.DriveBalance;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.AuxiliarIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PDP;
import frc.robot.subsystems.SuperiorIntake;
import frc.robot.subsystems.SuperiorIntake.Element;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Essa classe junta todos os elementos do  robo e  gerencia os funcionamento geral,
 * associando subsistemas e comandos
 */
public class RobotContainer {
  // Cria um objeto de cada subsistema
  private final Drivetrain drive = Drivetrain.getInstance();
  private final Arm arm  = Arm.getInstance();
  private final SuperiorIntake intake = SuperiorIntake.getInstance();
  private final PDP pdp = PDP.getInstance();
  private final AuxiliarIntake intakeAux = AuxiliarIntake.getInstance();
  private final Limelight limeLight = Limelight.getInstance();
  public static final ShuffleboardTab mainTab = Shuffleboard.getTab("Robot");

  /**
  * Cria os controles para comandar
  * Usamos um controle de Xbox ou manche para navegação
  * E uma central de botões  para os mecanismos
  **/
  private final XboxController m_driver = new XboxController(OperatorConstants.kDriverControllerPort);
  
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
  Trigger bResetOd = new JoystickButton(m_driver, 1);
  private Element m_autoSelectedElement;
  private final SendableChooser<Element> m_chooserElement = new SendableChooser<>();
  private static final String kNothingAuto = "nothing";
  private static final String kchargerStation = "station";
  private static final String kTaxi = "leave";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();




  /** 
   * Dentro do container é que são guardados organizados os elementos do robo.
   * Sao configurados os sistemas, associados os comandos aos botões, etc
   * */
  public RobotContainer() {
    m_chooserElement.setDefaultOption("Cone", Element.Cone);
    m_chooserElement.addOption("Cubo", Element.Cube);
    m_chooserElement.addOption("Nada", Element.None);
    SmartDashboard.putData("Elemento", m_chooserElement);
    m_chooser.setDefaultOption("Ficar Parado", kNothingAuto);
    m_chooser.addOption("Subir na Charger Station", kchargerStation);
    m_chooser.addOption("Sair da area", kTaxi);
    SmartDashboard.putData("Comportamento", m_chooser);





    // A função de Bindings que associa os eventos dos controles aos comandos
    configureBindings();
    arm.setMotorPower(-ArmConstants.kPowerWait);
  }

 
  /**
   * Aqui fazemos toda a configuração que associa eventos a comandos
   * Principais eventos são os comandos dos controles
   */
  private void configureBindings() {
    //Seta a navegação padrão pelo  controle
    drive.setDefaultCommand(new RunCommand(()->drive.setDriveMotors(m_driver.getLeftY()*-1, 
                              m_driver.getRightX()*-0.5), drive));
    bResetOd.onTrue(new DriveBalance());
    //drive.setDefaultCommand(new RunCommand(()->drive.setDriveMotors((m_driverController.getRawAxis(1)*((m_driverController.getRawAxis(3)+1)/2)), 
    //m_driverController.getRawAxis(2)*((m_driverController.getRawAxis(3)+1)/2)), drive));


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
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    m_autoSelectedElement = m_chooserElement.getSelected();
    System.out.println("element selected: " + m_autoSelectedElement.name());

    return new DriveAndTurn(2,90);
  }
}
