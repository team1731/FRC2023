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

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton adjustWheelEncoders = new JoystickButton(driver, XboxController.Button.kX.value);

  /* Subsystems */

  private final Swerve s_Swerve = new Swerve();
  private final PoseEstimatorSubsystem s_poseEstimatorSubsystem = new PoseEstimatorSubsystem(s_Swerve);

  // The container for the robot. Contains subsystems, OI devices, and commands. 
  public RobotContainer() {
	boolean fieldRelative = true;
    boolean openLoop = true;
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));

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
    zeroGyro.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));
    adjustWheelEncoders.whenPressed(new InstantCommand(() -> s_Swerve.adjustWheelEncoders()));
  }

  public Command getNamedAutonomousCommand(String autoCode) {
	switch (Character.getNumericValue(autoCode.charAt(0))) {
		case 0:
			return new _0_exampleAuto(s_Swerve, s_poseEstimatorSubsystem);
		case 9:
			return new _9_Move_Forward();
	}
	System.err.println("FATAL: SELECTED AUTO MODE " + autoCode + " DOES NOT MAP TO A KNOWN AUTONOMOUS CLASS -- DOING NOTHING!!!!");
    return null;
  }


	public void resetEncoders() {
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