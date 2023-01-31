// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AutoConstants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;
  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;
  private String autoCode = AutoConstants.kDEFAULT_AUTO_CODE;
  private String oldKeypadEntry = "";
  private String currentKeypadCommand = "";
  private NetworkTable keypad;
  private NetworkTable fieldInfo;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
	LiveWindow.disableAllTelemetry();
    ctreConfigs = new CTREConfigs();

	// Instantiate our robot container. This will perform all of our button bindings,
	// and put our autonomous chooser on the dashboard
	m_robotContainer = new RobotContainer();
	
	initSubsystems();

	SmartDashboard.putString(AutoConstants.kAUTO_CODE_KEY, AutoConstants.kDEFAULT_AUTO_CODE);
	SmartDashboard.putString("Build Info - Branch", "N/A");
	SmartDashboard.putString("Build Info - Commit Hash", "N/A");
	SmartDashboard.putString("Build Info - Date", "N/A");

	/*
	 * Note: do not think this is implemented in the gradle build, if we want to print this we will need to carry that over
	 */
	try {
		File buildInfoFile = new File(Filesystem.getDeployDirectory(), "DeployedBranchInfo.txt");
		if(buildInfoFile.exists() && buildInfoFile.canRead()){
			Scanner reader = new Scanner(buildInfoFile);
			int i = 0;
			while(reader.hasNext()){
				if(i == 0){
					SmartDashboard.putString("Build Info - Branch", reader.nextLine());
				} else if(i == 1){
					SmartDashboard.putString("Build Info - Commit Hash", reader.nextLine());
				} else {
					SmartDashboard.putString("Build Info - Date", reader.nextLine());
				}
				i++;
			}			
			reader.close();
		}
	} catch (FileNotFoundException fnf) {
		System.err.println("DeployedBranchInfo.txt not found");
		fnf.printStackTrace();
	}

	autoInitPreload(SmartDashboard.getString(AutoConstants.kAUTO_CODE_KEY, AutoConstants.kDEFAULT_AUTO_CODE));
	keypad = NetworkTableInstance.getDefault().getTable("KeyPad");
	fieldInfo = NetworkTableInstance.getDefault().getTable("FMSInfo");
  }
  
  private void autoInitPreload(String useCode) {
	m_autonomousCommand = null;

	System.out.println("Preloading AUTO CODE --> " + useCode);
	// 2023:
	// 2023: auto code is a single digit [0-9]
	// 2023:
	if(useCode == null || useCode.length() != 1 || !Character.isDigit(useCode.charAt(0))) {
		System.out.println("BAD AUTO CODE: " + useCode + " : DEFAULTING TO " + AutoConstants.kDEFAULT_AUTO_CODE);
		autoCode = AutoConstants.kDEFAULT_AUTO_CODE;
	}
	else{
		// NetworkTables called FMSInfo which contains an entry called IsRedAlliance
		boolean isRedAlliance = Boolean.valueOf(fieldInfo.getEntry("IsRedAlliance").getString("true"));
		m_autonomousCommand = m_robotContainer.getNamedAutonomousCommand(useCode, isRedAlliance);
		if(m_autonomousCommand != null){
			autoCode = useCode;
		}
		else{
			System.out.println("AUTO CODE " + useCode + " IS NOT IMPLEMENTED -- STAYING WITH AUTO CODE " + autoCode);
		}
	}
	SmartDashboard.putString(AutoConstants.kAUTO_CODE_KEY, autoCode);
	System.out.println("AUTO CODE being used by the software --> " + autoCode + "\n\n\n");
  }

  private void initSubsystems() {
	m_robotContainer.resetEncoders();
  }
  
  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

	//
	// read the KEYPAD, write to NETWORK TABLES
	//
	String newKeypadEntry = keypad.getEntry("driver entry").getString(oldKeypadEntry);
	if (!newKeypadEntry.equals(oldKeypadEntry)){
		System.out.println(".\n.\n.\nDRIVER ENTRY ==========================>>>>>>>> " + newKeypadEntry + "\n.\n.\n.");
		oldKeypadEntry = newKeypadEntry;
		SmartDashboard.putString("keypadCommand", newKeypadEntry);
	}

	m_robotContainer.displayEncoders();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
	SmartDashboard.putString(AutoConstants.kAUTO_CODE_KEY, autoCode);
	keypad.putValue("driver entry", NetworkTableValue.makeString(""));
  }

  @Override
  public void disabledPeriodic() {
    if (System.currentTimeMillis() % 5000 == 0) {
		// SmartDashboard.putBoolean("LowSensor", m_sequencer.lowSensorHasBall());
		// SmartDashboard.putBoolean("MidSensor", m_sequencer.midSensorHasBall());
		// SmartDashboard.putBoolean("HighSensor", m_sequencer.highSensorHasBall());
	}

	String newCode = SmartDashboard.getString(AutoConstants.kAUTO_CODE_KEY, autoCode);
	if(!newCode.equals(autoCode)) {
		System.out.println("New Auto Code read from dashboard. OLD: " + autoCode + ", NEW: " + newCode);
		autoInitPreload(newCode);
	}
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
	CommandScheduler.getInstance().cancelAll();

	if(m_autonomousCommand == null) {
		System.err.println("SOMETHING WENT WRONG - UNABLE TO RUN AUTONOMOUS! CHECK SOFTWARE!");
	} else {
		System.out.println("Running actual autonomous mode --> " + m_autonomousCommand.getClass().getSimpleName());
		m_robotContainer.zeroHeading();
		m_autonomousCommand.schedule();
	}		
	System.out.println("autonomousInit: End");
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
	CommandScheduler.getInstance().cancelAll();
	initSubsystems();
    // This makes sure that the autonomous stops running when
	// teleop starts running. If you want the autonomous to
	// continue until interrupted by another command, remove
	// this line or comment it out.
	if (m_autonomousCommand != null) {
		m_autonomousCommand.cancel();
	}
	currentKeypadCommand = "";
	SmartDashboard.getString("keypadCommand", currentKeypadCommand);
	keypad.putValue("driver entry", NetworkTableValue.makeString(""));
  }

  public static long millis = System.currentTimeMillis();

  public static boolean doSD() {
	  long now = System.currentTimeMillis();
	  if (now - millis > 300) {
		  millis = now;
		  return true;
	  }
	  return false;
  }


  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    String newKeypadCommand = SmartDashboard.getString("keypadCommand", currentKeypadCommand);
	if(!newKeypadCommand.equals(currentKeypadCommand)){
		// FEED FSM
		m_robotContainer.processKeypadCommand(newKeypadCommand);
		currentKeypadCommand = newKeypadCommand;
	}
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

}
