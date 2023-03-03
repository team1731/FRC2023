// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.state.arm.ArmSequence;
import frc.robot.state.arm.ArmStateMachine;
import frc.robot.subsystems.*;
import frc.robot.util.log.LogWriter;
import frc.robot.util.log.MessageLog;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConsoleConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.OpConstants.LedOption;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final CommandXboxController xboxController = new CommandXboxController(0);
  private final Joystick operator = new Joystick(1);


  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final Trigger kStart = xboxController.start();
  private final Trigger ky = xboxController.y();
  private final Trigger kb = xboxController.b();
  private final Trigger ka = xboxController.a();
  private final Trigger kx = xboxController.x();
  private final Trigger kLeftBumper = xboxController.leftBumper();
  private final Trigger kRightBumper = xboxController.rightBumper();
  private final Trigger kLeftTrigger = xboxController.leftTrigger();
  private final Trigger kRightTrigger = xboxController.rightTrigger();

  /* Operator Buttons */
  private final JoystickButton kPreventScoreBtn = new JoystickButton(operator,OperatorConsoleConstants.kPreventScoreBtnId);
  private final JoystickButton kExtraExtensionBtn = new JoystickButton(operator,OperatorConsoleConstants.kExtraExtensionBtnId);
  private final JoystickButton kReleaseBtn = new JoystickButton(operator,OperatorConsoleConstants.kReleaseBtnId);
  private final JoystickButton kIntakeBtn = new JoystickButton(operator,OperatorConsoleConstants.kIntakeBtnId);
  private final JoystickButton kKillSwitch = new JoystickButton(operator,OperatorConsoleConstants.kKillSwitchId);
  private final JoystickButton kAutoRecoverySwitch = new JoystickButton(operator,OperatorConsoleConstants.kAutoRecoverySwitchId);
  private final JoystickButton kConeSwitch = new JoystickButton(operator,OperatorConsoleConstants.kConeSwitchId);
  private final JoystickButton kCubeSwitch = new JoystickButton(operator,OperatorConsoleConstants.kCubeSwitchId);
  public final int kDisatalAxis = OperatorConsoleConstants.kDistalAxisId;
  public final int kProximalAxis = OperatorConsoleConstants.kProximalAxisId;


  /* Subsystems */
  private Swerve s_Swerve;
  private PoseEstimatorSubsystem s_poseEstimatorSubsystem;
  private ArmSubsystem s_armSubSystem;
  private ArmStateMachine sm_armStateMachine;
  private final LEDStringSubsystem m_ledstring;

  private GamePiece storedPiece; // used to temporarily store game piece setting when using cone flip feature

  // The container for the robot. Contains subsystems, OI devices, and commands. 
  public RobotContainer(
          Swerve swerve,
          PoseEstimatorSubsystem poseEstimatorSubsystem,
          ArmSubsystem armSubsystem,
          LEDStringSubsystem m_ledstring) {
    
	  boolean fieldRelative = true;
    boolean openLoop = false;
    s_Swerve = swerve;
    s_armSubSystem = armSubsystem;
    s_poseEstimatorSubsystem = poseEstimatorSubsystem;
    sm_armStateMachine = armSubsystem.getStateMachine();

    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, xboxController.getHID(), translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop)); 

    this.m_ledstring = m_ledstring;

    // Configure the button bindings
    configureButtonBindings();
  }
   

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    kStart.onTrue(new InstantCommand(() -> {
      s_Swerve.zeroGyro();
      s_Swerve.adjustWheelEncoders(); 
      s_armSubSystem.resetArmEncoders();
    }));
    ky.whileTrue((new ArmScoreCommand(sm_armStateMachine, ArmSequence.SCORE_HIGH, operator, kDisatalAxis)));
    kb.whileTrue((new ArmScoreCommand(sm_armStateMachine, ArmSequence.SCORE_MEDIUM, operator, kDisatalAxis)));
    ka.whileTrue((new ArmScoreCommand(sm_armStateMachine, ArmSequence.SCORE_LOW, operator, kDisatalAxis)));
    kx.whileTrue(new InstantCommand(() -> sm_armStateMachine.clearCurrentPath()));
    if(LogWriter.isArmRecordingEnabled()) {
      kLeftBumper.onTrue(new InstantCommand(() -> s_armSubSystem.startRecordingArmPath()));
      kRightBumper.onTrue(new InstantCommand(() -> s_armSubSystem.stopRecordingArmPath()));
    } else {
      kLeftBumper.whileTrue(new ArmPickupCommand(sm_armStateMachine, ArmSequence.PICKUP_HIGH, operator, kDisatalAxis));
      //kRightBumper.whileTrue(new ArmScoreCommand(sm_armStateMachine, ArmSequence.READ_KEYPAD, operator, kDisatalAxis));
    }
    kLeftTrigger.whileTrue(new ArmPickupCommand(sm_armStateMachine, ArmSequence.PICKUP_LOW, operator, kDisatalAxis));
    kRightTrigger.whileTrue(new FlipConeCommand(sm_armStateMachine));

    /* Operator Buttons */
    kPreventScoreBtn.whileTrue(new InstantCommand(() -> sm_armStateMachine.setAllowScore(false)));
    kPreventScoreBtn.whileFalse(new InstantCommand(() -> sm_armStateMachine.setAllowScore(true)));
    kExtraExtensionBtn.whileTrue(new InstantCommand(() -> sm_armStateMachine.setExtraExtension(true)));
    kExtraExtensionBtn.whileFalse(new InstantCommand(() -> sm_armStateMachine.setExtraExtension(false)));
    kIntakeBtn.whileTrue(new InstantCommand(() -> sm_armStateMachine.intake()));
    kIntakeBtn.whileFalse(new InstantCommand(() -> sm_armStateMachine.releaseIntake()));
    kReleaseBtn.whileTrue(new InstantCommand(() -> sm_armStateMachine.release()));
    kReleaseBtn.whileFalse(new InstantCommand(() -> sm_armStateMachine.stopRelease()));
    kKillSwitch.onTrue(new InstantCommand(() -> {
      sm_armStateMachine.addJoystickControl(operator, kProximalAxis, false);
      sm_armStateMachine.addJoystickControl(operator, kDisatalAxis, false);
      sm_armStateMachine.emergencyInterrupt();
    }));
    kAutoRecoverySwitch.onTrue(new InstantCommand(() -> sm_armStateMachine.attemptAutoRecovery()));
    kConeSwitch.onTrue(new InstantCommand(() -> {
      System.out.println("RobotContainer: Setting game piece to cone: " + storedPiece);
      sm_armStateMachine.setGamePiece(GamePiece.CONE);
      m_ledstring.setBlink(false);
      m_ledstring.setColor(LedOption.YELLOW);
    }));
    kCubeSwitch.onTrue(new InstantCommand(() -> {
      System.out.println("RobotContainer: Setting game piece to cube: " + storedPiece);
      sm_armStateMachine.setGamePiece(GamePiece.CUBE);
      m_ledstring.setBlink(false);
      m_ledstring.setColor(LedOption.PURPLE);
    }));
  }

  public Command getNamedAutonomousCommand(String autoCode, boolean isRedAlliance) {
    switch(autoCode) {
      case AutoConstants.kDefault:
        return new _9_Move_Forward(s_Swerve, s_poseEstimatorSubsystem);
      case AutoConstants.k_0_Example:
        return new _0_exampleAuto(s_Swerve, s_poseEstimatorSubsystem);
      case AutoConstants.k_Program_1:
        return new _Program_1(isRedAlliance, s_Swerve, s_poseEstimatorSubsystem, sm_armStateMachine);
      case AutoConstants.k_Program_2:
        return isRedAlliance()? new _Program_2R(isRedAlliance, s_Swerve, s_poseEstimatorSubsystem, sm_armStateMachine): new _Program_2(isRedAlliance, s_Swerve, s_poseEstimatorSubsystem, sm_armStateMachine);
      case AutoConstants.k_Program_3:
        return isRedAlliance()? new _Program_3R(isRedAlliance, s_Swerve, s_poseEstimatorSubsystem, sm_armStateMachine): new _Program_3(isRedAlliance, s_Swerve, s_poseEstimatorSubsystem, sm_armStateMachine);
      case AutoConstants.k_Program_4:
        return new _Program_4(isRedAlliance, s_Swerve, s_poseEstimatorSubsystem, sm_armStateMachine);
      case AutoConstants.k_9_Move_Forward:
				return new _9_Move_Forward(s_Swerve, s_poseEstimatorSubsystem);
		}
    System.err.println("FATAL: SELECTED AUTO MODE " + autoCode + " DOES NOT MAP TO A KNOWN AUTONOMOUS CLASS -- DOING NOTHING!!!!");
    return null;
  }


	public void resetEncoders() {
    s_armSubSystem.resetArmEncoders();
	}


	public void displayEncoders() {
	}


	public void zeroHeading() {
	}


	public void processKeypadCommand(String newKeypadCommand) {
		if(newKeypadCommand.length() > 0){
      System.out.println(newKeypadCommand + "\n");
      if (newKeypadCommand.toLowerCase().contains("cone")){
        m_ledstring.setBlink(false);
        m_ledstring.setColor(LedOption.YELLOW);
        sm_armStateMachine.setGamePiece(GamePiece.CONE);
        System.out.println("\n\nSHOWING YELLOW\n\n");
      }
      else if (newKeypadCommand.toLowerCase().contains("cube")){
        m_ledstring.setBlink(false);
        m_ledstring.setColor(LedOption.PURPLE);
        sm_armStateMachine.setGamePiece(GamePiece.CUBE);
        System.out.println("\n\nSHOWING PURPLE\n\n");
      }
      else if (newKeypadCommand.toLowerCase().contains("clear")){
        m_ledstring.setBlink(false);
        m_ledstring.setColor(LedOption.WHITE);
        sm_armStateMachine.setGamePiece(null);
        System.out.println("\n\nSHOWING WHITE\n\n");
     }
     // delegate to FSM
		 MessageLog.add("SENDING NEW COMMAND FROM NETWORK TABLES TO FSM: " + newKeypadCommand + "\n\n");
		}
	}

  private boolean isRedAlliance(){
    return DriverStation.getAlliance().equals(DriverStation.Alliance.Red);
    }
}