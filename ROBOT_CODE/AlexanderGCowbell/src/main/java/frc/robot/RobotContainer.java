// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OpConstants;
import frc.robot.Constants.OpConstants.KeypadControl;
import frc.robot.Constants.OpConstants.LedOption;

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
  private final LEDStringSubsystem m_ledstring;

  // The container for the robot. Contains subsystems, OI devices, and commands. 
  public RobotContainer(LEDStringSubsystem m_ledstring) {
	boolean fieldRelative = true;
    boolean openLoop = true;
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));

    this.m_ledstring = m_ledstring;

    // Configure the button bindings
    configureButtonBindings();
  }
   

  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    adjustWheelEncoders.onTrue(new InstantCommand(() -> s_Swerve.adjustWheelEncoders()));
  }

  public Command getNamedAutonomousCommand(String autoCode, boolean isRedAlliance) {
    switch(autoCode) {
      case AutoConstants.kDefault:
        return new _9_Move_Forward(s_Swerve, s_poseEstimatorSubsystem);
      case AutoConstants.k_0_Example:
        return new _0_exampleAuto(s_Swerve, s_poseEstimatorSubsystem);
      case AutoConstants.k_1_11Top_A_13Top_Drive_A:
        return new _1_11Top_A_13Top_Drive_A(isRedAlliance, s_Swerve, s_poseEstimatorSubsystem);
      case AutoConstants.k_2_13Top_B_Engage:
        return new _2_13Top_B_Engage(isRedAlliance, s_Swerve, s_poseEstimatorSubsystem);
      case AutoConstants.k_3_31Top_C_Engage:
        return new _3_31Top_C_Engage(isRedAlliance, s_Swerve, s_poseEstimatorSubsystem);
      case AutoConstants.k_4_33Top_D_31Top_Drive_D:
        return new _4_33Top_D_31Top_Drive_D(isRedAlliance, s_Swerve, s_poseEstimatorSubsystem);
      case AutoConstants.k_5_11Top_A_11Middle_Drive_A:
        return new _5_11Top_A_11Middle_Drive_A(isRedAlliance, s_Swerve, s_poseEstimatorSubsystem);
      case AutoConstants.k_6_33Top_D_33Middle_Drive_D:
        return new _6_33Top_D_33Middle_Drive_D(isRedAlliance, s_Swerve, s_poseEstimatorSubsystem);
      case AutoConstants.k_9_Move_Forward:
				return new _9_Move_Forward(s_Swerve, s_poseEstimatorSubsystem);
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
      System.out.println(newKeypadCommand + "\n");
		    switch(KeypadControl.valueOf(newKeypadCommand)){
			    case GET_CONE:
            m_ledstring.setBlink(false);
            m_ledstring.setColor(LedOption.YELLOW);
            System.out.println("\n\nSHOWING YELLOW\n\n");
				    break;
          case GET_CUBE:
            m_ledstring.setBlink(false);
            m_ledstring.setColor(LedOption.PURPLE);
            System.out.println("\n\nSHOWING PURPLE\n\n");
            break;
			  default:
				  break;
        }
			// delegate to FSM
			DataLogManager.log("SENDING NEW COMMAND FROM NETWORK TABLES TO FSM: " + newKeypadCommand + "\n\n");
		}
	}

}