// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final int distalAxis = XboxController.Axis.kRightY.value;

  /* Driver Buttons */
  private final Trigger kStart = xboxController.start();
  private final Trigger ky = xboxController.y();
  private final Trigger kb = xboxController.b();
  private final Trigger ka = xboxController.a();
  private final Trigger kLeftBumper = xboxController.leftBumper();
  private final Trigger kRightBumper = xboxController.rightBumper();
  private final Trigger kLeftTrigger = xboxController.leftTrigger();

  /* Operator Buttons */
  private final JoystickButton kPreventScoreBtn = new JoystickButton(operator,13);
  private final JoystickButton kReleaseBtn = new JoystickButton(operator,14);
  private final JoystickButton kIntakeBtn = new JoystickButton(operator,15);


  /* Subsystems */
  private Swerve s_Swerve;
  private PoseEstimatorSubsystem s_poseEstimatorSubsystem;
  private ArmSubsystem s_armSubSystem;
  private ArmStateMachine sm_armStateMachine;
  private final LEDStringSubsystem m_ledstring;

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
    //Test command to use joystick control of the arm
    //s_armSubSystem.setDefaultCommand(new TestArm(s_armSubSystem, xboxController.getHID(), translationAxis, distalAxis)); 
 

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
    ky.whileTrue((new ArmScoreCommand(sm_armStateMachine, ArmSequence.SCORE_HIGH)));
    kb.whileTrue((new ArmScoreCommand(sm_armStateMachine, ArmSequence.SCORE_MEDIUM)));
    ka.whileTrue((new ArmScoreCommand(sm_armStateMachine, ArmSequence.SCORE_LOW)));
    if(LogWriter.isArmRecordingEnabled()) {
      kLeftBumper.onTrue(new InstantCommand(() -> s_armSubSystem.startRecordingArmPath()));
      kRightBumper.onTrue(new InstantCommand(() -> s_armSubSystem.stopRecordingArmPath()));
    } else {
      kLeftBumper.whileTrue(new ArmPickupCommand(sm_armStateMachine, ArmSequence.PICKUP_HIGH));
      kRightBumper.whileTrue(new ArmScoreCommand(sm_armStateMachine, ArmSequence.READ_KEYPAD));
    }
    kLeftTrigger.whileTrue(new ArmPickupCommand(sm_armStateMachine, ArmSequence.PICKUP_LOW));

    /* Operator Buttons */
    kPreventScoreBtn.whileTrue(new InstantCommand(() -> sm_armStateMachine.setAllowScore(false)));
    kPreventScoreBtn.whileFalse(new InstantCommand(() -> sm_armStateMachine.setAllowScore(true)));
    kIntakeBtn.whileTrue(new InstantCommand(() -> sm_armStateMachine.intake()));
    kIntakeBtn.whileFalse(new InstantCommand(() -> sm_armStateMachine.stopIntake()));
    kReleaseBtn.whileTrue(new InstantCommand(() -> sm_armStateMachine.release()));
    kReleaseBtn.whileFalse(new InstantCommand(() -> sm_armStateMachine.stopRelease()));
  }

  public Command getNamedAutonomousCommand(String autoCode, boolean isRedAlliance) {
    switch(autoCode) {
      case AutoConstants.kDefault:
        return new _9_Move_Forward(s_Swerve, s_poseEstimatorSubsystem);
      case AutoConstants.k_0_Example:
        return new _0_exampleAuto(s_Swerve, s_poseEstimatorSubsystem);
      case AutoConstants.k_1_11Top_A_13Top_Drive_A:
        return new _1_11Top_A_13Top_Drive_A(isRedAlliance, s_Swerve, s_poseEstimatorSubsystem, sm_armStateMachine);
      case AutoConstants.k_2_13Top_B_Engage:
        return new _2_13Top_B_Engage(isRedAlliance, s_Swerve, s_poseEstimatorSubsystem, sm_armStateMachine);
      case AutoConstants.k_3_31Top_C_Engage:
        return new _3_31Top_C_Engage(isRedAlliance, s_Swerve, s_poseEstimatorSubsystem, sm_armStateMachine);
      case AutoConstants.k_4_33Top_D_31Top_Drive_D:
        return new _4_33Top_D_31Top_Drive_D(isRedAlliance, s_Swerve, s_poseEstimatorSubsystem, sm_armStateMachine);
      case AutoConstants.k_5_11Top_A_11Middle_Drive_A:
        return new _5_11Top_A_11Middle_Drive_A(isRedAlliance, s_Swerve, s_poseEstimatorSubsystem, sm_armStateMachine);
      case AutoConstants.k_6_33Top_D_33Middle_Drive_D:
        return new _6_33Top_D_33Middle_Drive_D(isRedAlliance, s_Swerve, s_poseEstimatorSubsystem, sm_armStateMachine);
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
}