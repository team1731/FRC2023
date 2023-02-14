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

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final int distalAxis = XboxController.Axis.kRightY.value;


  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton adjustAllEncoders = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton wristPos1 = new JoystickButton(driver,XboxController.Button.kA.value);
  private JoystickButton leftBumper = new JoystickButton(driver,XboxController.Button.kLeftBumper.value);
  private JoystickButton rightBumper = new JoystickButton(driver,XboxController.Button.kRightBumper.value);


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
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    adjustAllEncoders.onTrue(new InstantCommand(() -> {s_Swerve.adjustWheelEncoders(); s_armSubSystem.resetArmEncoders();}));
    wristPos1.onTrue(new InstantCommand(() -> s_armSubSystem.moveWrist(0.63)));
    wristPos1.onFalse(new InstantCommand(() -> s_armSubSystem.stopWrist()));

    if(LogWriter.isArmRecordingEnabled()) {
      leftBumper.onTrue(new InstantCommand(() -> s_armSubSystem.startRecordingArmPath()));
      rightBumper.onTrue(new InstantCommand(() -> s_armSubSystem.stopRecordingArmPath()));
    } else {
      leftBumper.whileTrue(new ArmPickupTestCommand(s_armSubSystem));
      rightBumper.whileTrue(new ArmScoreTestCommand(s_armSubSystem));
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