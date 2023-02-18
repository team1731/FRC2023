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

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.log.LogWriter;
import frc.robot.Constants.AutoConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final int distalAxis = XboxController.Axis.kRightY.value;


  /* Driver Buttons */
  private final JoystickButton kStart = new JoystickButton(driver, XboxController.Button.kStart.value);
  private final JoystickButton ky= new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton kx= new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton kb= new JoystickButton(driver,XboxController.Button.kB.value);
  private final JoystickButton ka = new JoystickButton(driver,XboxController.Button.kA.value);
  private JoystickButton leftBumper = new JoystickButton(driver,XboxController.Button.kLeftBumper.value);
  private JoystickButton rightBumper = new JoystickButton(driver,XboxController.Button.kRightBumper.value);
  private JoystickButton coneOrCube = new JoystickButton(operator,8);


  /* Subsystems */

  private Swerve s_Swerve;
  private PoseEstimatorSubsystem s_poseEstimatorSubsystem ;
  private ArmSubsystem s_armSubSystem ;

  // The container for the robot. Contains subsystems, OI devices, and commands. 
  public RobotContainer(
          Swerve swerve,
          PoseEstimatorSubsystem poseEstimatorSubsystem,
          ArmSubsystem armSubsystem) {
    
	  boolean fieldRelative = true;
    boolean openLoop = false;
    s_Swerve = swerve;
    s_armSubSystem = armSubsystem;
    s_poseEstimatorSubsystem = poseEstimatorSubsystem;
    // s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));
    // s_armSubSystem.setDefaultCommand(new TestArm(s_armSubSystem, driver, translationAxis, distalAxis)); 
 

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
    kStart.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    kStart.onTrue(new InstantCommand(() -> {s_Swerve.adjustWheelEncoders(); s_armSubSystem.resetArmEncoders();}));
    //intakeTest.onTrue(new InstantCommand(() -> s_armSubSystem.intake()));
    //intakeTest.onFalse(new InstantCommand(() -> s_armSubSystem.holdIntake()));
    //ejectTest.onTrue(new InstantCommand(() -> s_armSubSystem.eject()));
    //ejectTest.onFalse(new InstantCommand(() -> s_armSubSystem.stopIntake()));
    coneOrCube.whileTrue(new InstantCommand(() -> s_armSubSystem.setCone(false)));
    coneOrCube.whileFalse(new InstantCommand(() -> s_armSubSystem.setCone(true)));
    ky.whileTrue((new ArmScoreHighCommand(s_armSubSystem)));
    kb.whileTrue((new ArmScoreMediumCommand(s_armSubSystem)));
    ka.whileTrue((new ArmScoreLowCommand(s_armSubSystem)));

   // kb.whileTrue((new ArmScoreMid(s_armSubSystem)));
   // ka.whileTrue((new ArmScoreLow(s_armSubsystem)));
;
  
   // wristPos1.onTrue(new InstantCommand(() -> s_armSubSystem.moveWrist(0.63)));
   // wristPos1.onFalse(new InstantCommand(() -> s_armSubSystem.stopWrist()));

    if(LogWriter.isArmRecordingEnabled()) {
      leftBumper.onTrue(new InstantCommand(() -> s_armSubSystem.startRecordingArmPath()));
      rightBumper.onTrue(new InstantCommand(() -> s_armSubSystem.stopRecordingArmPath()));
    } else {
      leftBumper.whileTrue(new ArmPickupHighCommand(s_armSubSystem));
      rightBumper.whileTrue(new ArmPickupLowCommand(s_armSubSystem));
    }
  }

  public Command getNamedAutonomousCommand(String autoCode, boolean isRedAlliance) {
    switch(autoCode) {
      case AutoConstants.k_0_Example:
        return new _0_exampleAuto(s_Swerve, s_poseEstimatorSubsystem);
      case AutoConstants.k_1_11Top_A_13Top_Drive_A:
        return new _1_11Top_A_13Top_Drive_A(isRedAlliance, s_Swerve, s_poseEstimatorSubsystem);
      case AutoConstants.k_2_13Top_B_Engage:
        return new _2_13Top_B_Engage(isRedAlliance, s_Swerve, s_poseEstimatorSubsystem);
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
			// delegate to FSM
			System.out.println("SENDING NEW COMMAND FROM NETWORK TABLES TO FSM: " + newKeypadCommand + "\n\n");
		}
	}
}