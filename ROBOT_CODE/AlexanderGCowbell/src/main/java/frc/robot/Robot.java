// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.LogConstants;
import frc.robot.state.arm.ArmStateMachine;
import frc.robot.util.log.LogWriter;
import frc.robot.util.log.MessageLog;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDStringSubsystem;

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
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private String autoCode = AutoConstants.kDefault;
  private String oldKeypadEntry = "";
  private String currentKeypadCommand = "";
  private NetworkTable keypad;
  private NetworkTable fieldInfo;
  private boolean isRedAlliance = true;
  private int stationNumber = 0;
  public static long millis = System.currentTimeMillis();
  private Swerve s_Swerve;
  private PoseEstimatorSubsystem s_poseEstimatorSubsystem;
  private ArmSubsystem s_armSubSystem;
  private ArmStateMachine sm_armStateMachine;

  public Robot() {
	if(LogWriter.isArmRecordingEnabled()) {
		addPeriodic(() -> {
			if(s_armSubSystem.isArmRecordingRunning()) {
				s_armSubSystem.writeArmPathValues();
			}
		}, LogConstants.recordingPeriod, LogConstants.recordingOffset);
	}
  }

  // SUBSYSTEM DECLARATION
  private LEDStringSubsystem m_ledstring;
  private boolean ledBlinking;

  // NOTE: FOR TESTING PURPOSES ONLY!
  private final Joystick driver = new Joystick(0);
  private final JoystickButton blinker = null; //new JoystickButton(driver, XboxController.Button.kX.value);


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   * 
   * NOTE: ASCII ART from https://textfancy.com/text-art/  ("small negative")
   */
//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   ██ ▄▄▀██ ▄▄▄ ██ ▄▄▀██ ▄▄▄ █▄▄ ▄▄███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄
//   ██ ▀▀▄██ ███ ██ ▄▄▀██ ███ ███ ██████ ███ █ █ ██ ████ ██
//   ██ ██ ██ ▀▀▀ ██ ▀▀ ██ ▀▀▀ ███ █████▀ ▀██ ██▄ █▀ ▀███ ██
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void robotInit() {
	LogWriter.setupLogging();
	MessageLog.start();
	MessageLog.add("\n\n\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  EVENT: " + DriverStation.getEventName() + " <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n\n");

	LiveWindow.disableAllTelemetry();
    ctreConfigs = new CTREConfigs();

	s_Swerve = new Swerve();
  	s_poseEstimatorSubsystem = new PoseEstimatorSubsystem(s_Swerve);
  	s_armSubSystem = new ArmSubsystem();
	sm_armStateMachine = s_armSubSystem.getStateMachine();
	m_ledstring = new LEDStringSubsystem();

	// Instantiate our robot container. This will perform all of our button bindings,
	// and put our autonomous chooser on the dashboard
	m_robotContainer = new RobotContainer(s_Swerve, s_poseEstimatorSubsystem, s_armSubSystem, m_ledstring);

	initSubsystems();

	autoChooser.setDefaultOption(AutoConstants.kDefault,             AutoConstants.kDefault);
	autoChooser.addOption(       AutoConstants.k_0_Example,          AutoConstants.k_0_Example);
	autoChooser.addOption(       AutoConstants.k_Program_1,          AutoConstants.k_Program_1);
	autoChooser.addOption(       AutoConstants.k_Program_2,          AutoConstants.k_Program_2);
	autoChooser.addOption(       AutoConstants.k_Program_3,          AutoConstants.k_Program_3);
	autoChooser.addOption(       AutoConstants.k_Program_4,          AutoConstants.k_Program_4);
	autoChooser.addOption(       AutoConstants.k_9_Move_Forward,     AutoConstants.k_9_Move_Forward);
    SmartDashboard.putData(AutoConstants.kAutoCodeKey, autoChooser);
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

	autoInitPreload();
	keypad = NetworkTableInstance.getDefault().getTable("KeyPad");

	//For testing LED Blinking only. The arm will set blink true after a piece has been secured.
	//ledBlinking = true;
	//blinker.onTrue(new InstantCommand(() -> {m_ledstring.setBlink(ledBlinking); ledBlinking = !ledBlinking;}));
  }
  

//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █▄ ▄██ ▄▄▄ ████ ▄▄▀██ ▄▄▄██ ▄▄▀███ ▄▄▀██ █████ ████▄ ▄█ ▄▄▀██ ▀██ ██ ▄▄▀██ ▄▄▄
//   ██ ███▄▄▄▀▀████ ▀▀▄██ ▄▄▄██ ██ ███ ▀▀ ██ █████ █████ ██ ▀▀ ██ █ █ ██ █████ ▄▄▄
//   █▀ ▀██ ▀▀▀ ████ ██ ██ ▀▀▀██ ▀▀ ███ ██ ██ ▀▀ ██ ▀▀ █▀ ▀█ ██ ██ ██▄ ██ ▀▀▄██ ▀▀▀
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  private boolean isRedAlliance(){
	return DriverStation.getAlliance().equals(DriverStation.Alliance.Red);
  }


// ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
// ██ ▄▄ ██ ▄▄▄█▄▄ ▄▄████ ▄▄▄ █▄▄ ▄▄█ ▄▄▀█▄▄ ▄▄█▄ ▄██ ▄▄▄ ██ ▀██ ████ ▀██ ██ ██ ██ ▄▀▄ ██ ▄▄▀██ ▄▄▄██ ▄▄▀
// ██ █▀▀██ ▄▄▄███ ██████▄▄▄▀▀███ ███ ▀▀ ███ ████ ███ ███ ██ █ █ ████ █ █ ██ ██ ██ █ █ ██ ▄▄▀██ ▄▄▄██ ▀▀▄
// ██ ▀▀▄██ ▀▀▀███ ██████ ▀▀▀ ███ ███ ██ ███ ███▀ ▀██ ▀▀▀ ██ ██▄ ████ ██▄ ██▄▀▀▄██ ███ ██ ▀▀ ██ ▀▀▀██ ██ 
// ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  private int getStationNumber(){
	return DriverStation.getLocation();
  }


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █ ▄▄▀██ ██ █▄▄ ▄▄██ ▄▄▄ ███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄████ ▄▄ ██ ▄▄▀██ ▄▄▄██ █████ ▄▄▄ █ ▄▄▀██ ▄▄▀
//   █ ▀▀ ██ ██ ███ ████ ███ ████ ███ █ █ ██ ████ ██████ ▀▀ ██ ▀▀▄██ ▄▄▄██ █████ ███ █ ▀▀ ██ ██ 
//   █ ██ ██▄▀▀▄███ ████ ▀▀▀ ███▀ ▀██ ██▄ █▀ ▀███ ██████ █████ ██ ██ ▀▀▀██ ▀▀ ██ ▀▀▀ █ ██ ██ ▀▀ 
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  private void autoInitPreload() {
	m_autonomousCommand = null;

	String useCode = autoChooser.getSelected();

    MessageLog.add("\nPreloading AUTO CODE --> " + useCode);

	if(useCode == null) {
        MessageLog.add("\nNULL AUTO CODE : DEFAULTING TO " + AutoConstants.kDefault);
		autoCode = AutoConstants.kDefault;
	}
	else{
		m_autonomousCommand = m_robotContainer.getNamedAutonomousCommand(useCode, isRedAlliance);
		if(m_autonomousCommand != null){
			autoCode = useCode;
			MessageLog.add("\n=====>>> PRELOADED AUTONOMOUS ROUTINE: " + m_autonomousCommand.getClass().getName() + " " + (isRedAlliance?"RED":"BLUE") + " <<<=====");
		}
		else{
			MessageLog.add("\nAUTO CODE " + useCode + " IS NOT IMPLEMENTED -- STAYING WITH AUTO CODE " + autoCode);
		}
	}
    MessageLog.add("\nAUTO CODE being used by the software --> " + autoCode + "\n");
  }


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄████ ▄▄▄ ██ ██ ██ ▄▄▀██ ▄▄▄ ██ ███ ██ ▄▄▄ █▄▄ ▄▄██ ▄▄▄██ ▄▀▄ ██ ▄▄▄ 
//   ██ ███ █ █ ██ ████ ██████▄▄▄▀▀██ ██ ██ ▄▄▀██▄▄▄▀▀██▄▀▀▀▄██▄▄▄▀▀███ ████ ▄▄▄██ █ █ ██▄▄▄▀▀
//   █▀ ▀██ ██▄ █▀ ▀███ ██████ ▀▀▀ ██▄▀▀▄██ ▀▀ ██ ▀▀▀ ████ ████ ▀▀▀ ███ ████ ▀▀▀██ ███ ██ ▀▀▀ 
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  private void initSubsystems() {
	m_ledstring.init();
	m_robotContainer.resetEncoders();
  }

  

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
// ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
// ██ ▄▄▀██ ▄▄▄ ██ ▄▄▀██ ▄▄▄ █▄▄ ▄▄████ ▄▄ ██ ▄▄▄██ ▄▄▀█▄ ▄██ ▄▄▄ ██ ▄▄▀█▄ ▄██ ▄▄▀
// ██ ▀▀▄██ ███ ██ ▄▄▀██ ███ ███ ██████ ▀▀ ██ ▄▄▄██ ▀▀▄██ ███ ███ ██ ██ ██ ███ ███
// ██ ██ ██ ▀▀▀ ██ ▀▀ ██ ▀▀▀ ███ ██████ █████ ▀▀▀██ ██ █▀ ▀██ ▀▀▀ ██ ▀▀ █▀ ▀██ ▀▀▄
// ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
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
        MessageLog.add(".\n.\n.\nDRIVER ENTRY ==========================>>>>>>>> " + newKeypadEntry + "\n.\n.\n.");
		oldKeypadEntry = newKeypadEntry;
		SmartDashboard.putString("keypadCommand", newKeypadEntry);
		m_robotContainer.processKeypadCommand(newKeypadEntry);
		sm_armStateMachine.setKeyedSequence(newKeypadEntry);
	}

	m_robotContainer.displayEncoders();
  }


/** This function is called once each time the robot enters Disabled mode. */
//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   ██ ▄▄▀█▄ ▄██ ▄▄▄ █ ▄▄▀██ ▄▄▀██ █████ ▄▄▄██ ▄▄▀███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄
//   ██ ██ ██ ███▄▄▄▀▀█ ▀▀ ██ ▄▄▀██ █████ ▄▄▄██ ██ ████ ███ █ █ ██ ████ ██
//   ██ ▀▀ █▀ ▀██ ▀▀▀ █ ██ ██ ▀▀ ██ ▀▀ ██ ▀▀▀██ ▀▀ ███▀ ▀██ ██▄ █▀ ▀███ ██
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void disabledInit() {
	keypad.putValue("driver entry", NetworkTableValue.makeString(""));
	sm_armStateMachine.disabledInit();
	s_armSubSystem.initializeArmPositions();
	s_armSubSystem.resetArmEncoders();

  }


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   ██ ▄▄▀█▄ ▄██ ▄▄▄ █ ▄▄▀██ ▄▄▀██ █████ ▄▄▄██ ▄▄▀████ ▄▄ ██ ▄▄▄██ ▄▄▀█▄ ▄██ ▄▄▄ ██ ▄▄▀█▄ ▄██ ▄▄▀
//   ██ ██ ██ ███▄▄▄▀▀█ ▀▀ ██ ▄▄▀██ █████ ▄▄▄██ ██ ████ ▀▀ ██ ▄▄▄██ ▀▀▄██ ███ ███ ██ ██ ██ ███ ███
//   ██ ▀▀ █▀ ▀██ ▀▀▀ █ ██ ██ ▀▀ ██ ▀▀ ██ ▀▀▀██ ▀▀ ████ █████ ▀▀▀██ ██ █▀ ▀██ ▀▀▀ ██ ▀▀ █▀ ▀██ ▀▀▄
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void disabledPeriodic() {
	//s_armSubSystem.resetArmEncoders();
    if (System.currentTimeMillis() % 5000 == 0) {
		// SmartDashboard.putBoolean("LowSensor", m_sequencer.lowSensorHasBall());
		// SmartDashboard.putBoolean("MidSensor", m_sequencer.midSensorHasBall());
		// SmartDashboard.putBoolean("HighSensor", m_sequencer.highSensorHasBall());
	}

	String newCode = autoChooser.getSelected();
	if(!newCode.equals(autoCode)) {
        MessageLog.add("New Auto Code read from dashboard. OLD: " + autoCode + ", NEW: " + newCode);
		autoInitPreload();
	}

	boolean isRedAlliance = isRedAlliance();
	if(this.isRedAlliance != isRedAlliance){
		this.isRedAlliance = isRedAlliance;
        MessageLog.add("\n\n===============>>>>>>>>>>>>>>  WE ARE " + (isRedAlliance?"RED":"BLUE") + " ALLIANCE  <<<<<<<<<<<<=========================");
		this.autoInitPreload();
	}

	int stationNumber = getStationNumber();
	if(this.stationNumber != stationNumber){
		this.stationNumber = stationNumber;
        MessageLog.add("===============>>>>>>>>>>>>>>  WE ARE STATION NUMBER " + stationNumber + "  <<<<<<<<<<<<=========================\n");
	}
  }


/** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █ ▄▄▀██ ██ █▄▄ ▄▄██ ▄▄▄ ██ ▀██ ██ ▄▄▄ ██ ▄▀▄ ██ ▄▄▄ ██ ██ ██ ▄▄▄ ███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄
//   █ ▀▀ ██ ██ ███ ████ ███ ██ █ █ ██ ███ ██ █ █ ██ ███ ██ ██ ██▄▄▄▀▀████ ███ █ █ ██ ████ ██
//   █ ██ ██▄▀▀▄███ ████ ▀▀▀ ██ ██▄ ██ ▀▀▀ ██ ███ ██ ▀▀▀ ██▄▀▀▄██ ▀▀▀ ███▀ ▀██ ██▄ █▀ ▀███ ██
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void autonomousInit() {
    MessageLog.add("AUTO INIT");
	CommandScheduler.getInstance().cancelAll();

	if(m_autonomousCommand == null) {
		System.err.println("SOMETHING WENT WRONG - UNABLE TO RUN AUTONOMOUS! CHECK SOFTWARE!");
	} else {
        MessageLog.add("------------> RUNNING AUTONOMOUS COMMAND: " + m_autonomousCommand.getClass().getSimpleName() + " <----------");
		m_robotContainer.zeroHeading();
		m_autonomousCommand.schedule();
	}
    MessageLog.add("autonomousInit: End");
  }


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █ ▄▄▀██ ██ █▄▄ ▄▄██ ▄▄▄ ██ ▀██ ██ ▄▄▄ ██ ▄▀▄ ██ ▄▄▄ ██ ██ ██ ▄▄▄ ████ ▄▄ ██ ▄▄▄██ ▄▄▀█▄ ▄██ ▄▄▄ ██ ▄▄▀█▄ ▄██ ▄▄▀
//   █ ▀▀ ██ ██ ███ ████ ███ ██ █ █ ██ ███ ██ █ █ ██ ███ ██ ██ ██▄▄▄▀▀████ ▀▀ ██ ▄▄▄██ ▀▀▄██ ███ ███ ██ ██ ██ ███ ███
//   █ ██ ██▄▀▀▄███ ████ ▀▀▀ ██ ██▄ ██ ▀▀▀ ██ ███ ██ ▀▀▀ ██▄▀▀▄██ ▀▀▀ ████ █████ ▀▀▀██ ██ █▀ ▀██ ▀▀▀ ██ ▀▀ █▀ ▀██ ▀▀▄
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void autonomousPeriodic() {
    if(doSD()){ MessageLog.add("AUTO PERIODIC");}
  }


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █▄▄ ▄▄██ ▄▄▄██ █████ ▄▄▄██ ▄▄▄ ██ ▄▄ ███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄
//   ███ ████ ▄▄▄██ █████ ▄▄▄██ ███ ██ ▀▀ ████ ███ █ █ ██ ████ ██
//   ███ ████ ▀▀▀██ ▀▀ ██ ▀▀▀██ ▀▀▀ ██ ██████▀ ▀██ ██▄ █▀ ▀███ ██
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void teleopInit() {	
	// Record both DS control and joystick data in TELEOP
	MessageLog.getLogger();
	
    MessageLog.add("TELEOP INIT");
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
  }


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   ██ ▄▄▀██ ▄▄▄ ████ ▄▄▄ ██ ▄▄▀
//   ██ ██ ██ ███ ████▄▄▄▀▀██ ██ 
//   ██ ▀▀ ██ ▀▀▀ ████ ▀▀▀ ██ ▀▀ 
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  public static boolean doSD() {
	long now = System.currentTimeMillis();
	if (now - millis > 1000) {
		MessageLog.getLogger().flush();
		millis = now;
		return true;
	}
	return false;
  }

/** This function is called periodically during operator control. */
//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █▄▄ ▄▄██ ▄▄▄██ █████ ▄▄▄██ ▄▄▄ ██ ▄▄ ████ ▄▄ ██ ▄▄▄██ ▄▄▀█▄ ▄██ ▄▄▄ ██ ▄▄▀█▄ ▄██ ▄▄▀
//   ███ ████ ▄▄▄██ █████ ▄▄▄██ ███ ██ ▀▀ ████ ▀▀ ██ ▄▄▄██ ▀▀▄██ ███ ███ ██ ██ ██ ███ ███
//   ███ ████ ▀▀▀██ ▀▀ ██ ▀▀▀██ ▀▀▀ ██ ███████ █████ ▀▀▀██ ██ █▀ ▀██ ▀▀▀ ██ ▀▀ █▀ ▀██ ▀▀▄
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void teleopPeriodic() {
    if(doSD()){ MessageLog.add("TELEOP PERIODIC");}
    String newKeypadCommand = SmartDashboard.getString("keypadCommand", currentKeypadCommand);
	if(!newKeypadCommand.equals(currentKeypadCommand)){
		// FEED FSM
		m_robotContainer.processKeypadCommand(newKeypadCommand);
		sm_armStateMachine.setKeyedSequence(newKeypadCommand);
		currentKeypadCommand = newKeypadCommand;
	}

	if(!ledBlinking && sm_armStateMachine.isHoldingGamePiece()) {
		m_ledstring.setBlink(true);
		ledBlinking = true;
	} else if(ledBlinking && !sm_armStateMachine.isHoldingGamePiece()) {
		m_ledstring.setBlink(false);
		ledBlinking = false;
	}
  }


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █▄▄ ▄▄██ ▄▄▄██ ▄▄▄ █▄▄ ▄▄███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄
//   ███ ████ ▄▄▄██▄▄▄▀▀███ ██████ ███ █ █ ██ ████ ██
//   ███ ████ ▀▀▀██ ▀▀▀ ███ █████▀ ▀██ ██▄ █▀ ▀███ ██
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }


  /** This function is called periodically during test mode. */
//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █▄▄ ▄▄██ ▄▄▄██ ▄▄▄ █▄▄ ▄▄████ ▄▄ ██ ▄▄▄██ ▄▄▀█▄ ▄██ ▄▄▄ ██ ▄▄▀█▄ ▄██ ▄▄▀
//   ███ ████ ▄▄▄██▄▄▄▀▀███ ██████ ▀▀ ██ ▄▄▄██ ▀▀▄██ ███ ███ ██ ██ ██ ███ ███
//   ███ ████ ▀▀▀██ ▀▀▀ ███ ██████ █████ ▀▀▀██ ██ █▀ ▀██ ▀▀▀ ██ ▀▀ █▀ ▀██ ▀▀▄
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void testPeriodic() {

  }

}