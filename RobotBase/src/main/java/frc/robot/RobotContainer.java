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
import frc.robot.subsystems.drive.DriveSubsystem;

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
	private DriveSubsystem m_drive;

  /** The container for the robot. Contains subsystems, OI devices, and commands. 
   * @param m_vision*/
  public RobotContainer(DriveSubsystem drive, VisionSubsystem m_vision) {
    this.m_drive = drive;
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new exampleAuto(s_Swerve,s_poseEstimatorSubsystem);
  }

  public _NamedAutoMode getNamedAutonomousCommand(String autoSelected) {
		String autoMode = "";
		int initialDelaySeconds = 0;
		int secondaryDelaySeconds = 0;
		if (autoSelected.length() > 1) {
			autoMode = autoSelected.substring(0, 2);
		}
		if (autoSelected.length() > 2) {
			try {
				initialDelaySeconds = Integer.parseInt(autoSelected.substring(2, 2));
			} catch (Exception e) {
				System.out.println("INITIAL DELAY did not parse -- defaulting to 0 seconds!!!");
			}
		}
		if (autoSelected.length() > 3) {
			try {
				secondaryDelaySeconds = Integer.parseInt(autoSelected.substring(3, 3));
			} catch (Exception e) {
				System.out.println("SECONDARY DELAY did not parse -- defaulting to 0 seconds!!!");
			}
		}

		_NamedAutoMode selectedAutoMode = null;

		try {
			selectedAutoMode = createNamedAutoMode(autoMode);
		} catch (_NotImplementedProperlyException e) {
			System.err.println("SELECTED MODE NOT IMPLEMENTED -- DEFAULT TO F1_MOVE_FORWARD!!!");
			try {
				selectedAutoMode = new _NamedAutoMode(new F1_Move_Forward(m_drive));
			} catch (_NotImplementedProperlyException e2) {
				System.err.println("F1_Move_Forward could NOT be created -- Aborting!!!");
				return null;
			}
		}
		if (selectedAutoMode != null) {
			selectedAutoMode.delayableStrafingAutoMode.setInitialDelaySeconds(initialDelaySeconds);
			selectedAutoMode.delayableStrafingAutoMode.setSecondaryDelaySeconds(secondaryDelaySeconds);
		}

		return selectedAutoMode;
	}

	private _NamedAutoMode createNamedAutoMode(String autoModeName) throws _NotImplementedProperlyException {
		switch (autoModeName) {
			case "F1":
				return new _NamedAutoMode(new F1_Move_Forward(m_drive));
			// case "L2":
			//     return new _NamedAutoMode(new L2_B3X2(m_drive, m_intake, m_launch));
			// case "L3":
			//     return new _NamedAutoMode(new L3_B3X2_B5X2(m_drive, m_intake, m_launch));
			// case "L4":
			//     return new _NamedAutoMode(new L4_B3L2_B5B4L2(m_drive, m_intake, m_launch));
			// case "L5":
			//     return new _NamedAutoMode(new L5_B3X2_B5B4X2_B2B1X1(m_drive, m_intake, m_launch));
			// case "L6":
			//     return new _NamedAutoMode(new L6_B3X2_B5B4X2_B2X1(m_drive, m_intake, m_launch));
			// case "R2":
			//     return new _NamedAutoMode(new R2_B1X2(m_drive, m_intake, m_launch));
			// case "R5":
			//     return new _NamedAutoMode(new R5_X2B2X1B4B5X2(m_drive, m_intake, m_launch));
			// case "C2":
			//     return new _NamedAutoMode(new C2_B2X2(m_drive, m_intake, m_launch));
			// case "C1":
			//     return new _NamedAutoMode(new C1_B2X2(m_drive, m_intake, m_launch));
			// case "X0":
			// 	return new _NamedAutoMode(new X0_DoNothing(m_drive, m_intake, m_launch));
			// case "D2":
			// 	return new _NamedAutoMode(new D2_B3X2(m_drive, m_intake, m_launch));
			// case "D1":
			// 	return new _NamedAutoMode(new D1_B3X2(m_drive, m_intake, m_launch));
		
			case "C4":
			default:
				return new _NamedAutoMode(new F1_Move_Forward(m_drive));
		}
	}

}
