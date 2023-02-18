package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motion.*;
import com.ctre.phoenix.ErrorCode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.state.arm.ArmStateMachine;
import frc.robot.Constants.ArmConstants;
import frc.data.mp.*;
import frc.data.mp.ArmPath.ArmMotor;
import frc.data.mp.ArmPath.Direction;
import frc.robot.util.ArbitraryFeedForward;
import frc.robot.util.log.Logger;
import frc.robot.util.log.LogWriter;
import frc.robot.util.log.LogWriter.Log;
import frc.robot.util.log.loggers.ArmPathRecording;

public class ArmSubsystem extends SubsystemBase {
    private ArmStateMachine stateMachine;

    // motors for the arm
    private WPI_TalonFX proximalMotor;
    private WPI_TalonFX distalMotor;
    private AnalogInput distalAbsolute;
    private AnalogInput proximalAbsolute;

    // motion profiles/buffers for arm proximal and distal motors
    private BufferedTrajectoryPointStream proximalBufferedStream;
    private BufferedTrajectoryPointStream distalBufferedStream;

    // motors for the wrist/intake
    private CANSparkMax wristMotor;
    private CANSparkMax intakeMotor;
    private SparkMaxPIDController wristPIDController; 
//private SparkMaxPIDController intakePIDController;
    private AbsoluteEncoder wristEncoder;

    // arm recording
    Logger armPathLogger;
    
    // state tracking
    private ArmPath currentPath = null;
    private Direction currentDirection = null;
    private double pathStartedTime = 0;
    private int profileStartIndex = 0;
    private boolean proximalMotorRunning = false;
    private boolean distalMotorRunning = false;
    private boolean wristFlexed = false; // flexed is hand bent down (scoring/pickup), extended is hand bent up (home/carrying position)
    private boolean ejecting = false;
    private boolean intaking = false;
    private Double intakingTimer = null;
    private boolean cone = false;
    private boolean goHome = true;


    public ArmSubsystem() {
        stateMachine = new ArmStateMachine(this);
        initializeArmMotors();
        distalAbsolute = new AnalogInput(0);
        proximalAbsolute = new AnalogInput(1);
        armPathLogger = null;
    }

    public ArmStateMachine getStateMachine() {
        return stateMachine;
    }
    
    private void initializeArmMotors() {
        proximalMotor = new WPI_TalonFX(ArmConstants.proximalCancoderId, "canivore1");
        initializeTalonMotor(proximalMotor, TalonFXInvertType.CounterClockwise);

        distalMotor = new WPI_TalonFX(ArmConstants.distalCancoderId, "canivore1");
        initializeTalonMotor(distalMotor, TalonFXInvertType.CounterClockwise);

        wristMotor = new CANSparkMax(ArmConstants.wristCancoderId, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(ArmConstants.intakeCancoderId, MotorType.kBrushless);

        initializeWrist();
        initializeIntake();
    }


    private void initializeTalonMotor(WPI_TalonFX motor, TalonFXInvertType invertType) {
        TalonFXConfiguration talonConfig = new TalonFXConfiguration(); // factory default settings
        talonConfig.neutralDeadband = ArmConstants.kNeutralDeadband; /* 0.1 % super small for best low-speed control */
        talonConfig.slot0.kF = ArmConstants.kGains_MotProf.kF;
        talonConfig.slot0.kP = ArmConstants.kGains_MotProf.kP;
        talonConfig.slot0.kI = ArmConstants.kGains_MotProf.kI;
        talonConfig.slot0.kD = ArmConstants.kGains_MotProf.kD;
        talonConfig.slot0.integralZone = (int) ArmConstants.kGains_MotProf.kIzone;
        talonConfig.slot0.closedLoopPeakOutput = ArmConstants.kGains_MotProf.kPeakOutput;
        // talonConfig.slot0.allowableClosedloopError // left default for this example
        // talonConfig.slot0.maxIntegralAccumulator; // left default for this example
        // talonConfig.slot0.closedLoopPeriod; // left default for this example
        motor.configAllSettings(talonConfig);

        // Note: these two lines required for motion magic to work
        motor.configMotionCruiseVelocity(15000, 30);
		motor.configMotionAcceleration(6000, 30);

        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
        motor.configNominalOutputForward(0, 30);
		motor.configNominalOutputReverse(0, 30);
		motor.configPeakOutputForward(0.5, 30);
		motor.configPeakOutputReverse(-0.5, 30);
        // motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 30, 0.2));
        motor.setInverted(invertType);
    }

    // home for any logic needed to reset, especially when robot moves to disabled state
    // ensures motion profiles are cleared so motor doesn't try to process last profile when re-enabled
    // moves the arm into a home (safe) position
    public void reset() {
        proximalMotor.clearMotionProfileTrajectories();
        distalMotor.clearMotionProfileTrajectories();

        if(LogWriter.isArmRecordingEnabled()) {
            // disengage the arm and wrist motors so both can be moved freely for recording
            stopIntake();
            stopWrist();
            proximalMotor.set(ControlMode.PercentOutput, 0);
            distalMotor.set(ControlMode.PercentOutput, 0);
        } else {
            // move the arm into normal home (safe) position
            stopIntake();
            moveWristHome();
            moveProximalArmHome();
            moveDistalArmHome();
        }        
    }

    public void startArmMovement(ArmPath armPath) {
        // init motion profile buffers for proximal/distal motors
        currentPath = armPath;
        currentDirection = Direction.FORWARD;
        profileStartIndex = 0;
        proximalBufferedStream = armPath.getInitializedBuffer(ArmMotor.PROXIMAL, 0, currentDirection);
        distalBufferedStream = armPath.getInitializedBuffer(ArmMotor.DISTAL, 0, currentDirection);

        // start arm movement
        moveArm();
    }

    public void restartArmMovement(int startIndex) {
        // re-init motion profile buffers for proximal/distal motors, 
        currentDirection = Direction.FORWARD;
        proximalBufferedStream = currentPath.getInitializedBuffer(ArmMotor.PROXIMAL, startIndex, currentDirection);
        distalBufferedStream = currentPath.getInitializedBuffer(ArmMotor.DISTAL, startIndex, currentDirection);

        // start arm movement
        moveArm();
    }

    public void reverseArmMovment() {
        boolean completedPath = !isArmMoving();

        // update profile buffers with a reverse path

        int startPosition = 0;
        int pointsLastIndex = currentPath.getNumberOfPoints()-1;
        if(completedPath) { 
            // not moving, we finished our path, run the whole thing back
            startPosition = pointsLastIndex;
        } else { 
            // we are still in the middle of our path, first stop current movement
            stopArm();
            // now, see how far along and run back from where we stopped
            startPosition = getPathIndex();
        }

        currentDirection = Direction.REVERSE;
        profileStartIndex = startPosition;
        proximalBufferedStream = currentPath.getInitializedBuffer(ArmMotor.PROXIMAL, startPosition, currentDirection);
        distalBufferedStream = currentPath.getInitializedBuffer(ArmMotor.DISTAL, startPosition, currentDirection);

        // restart arm movement
        moveArm();
    }

    private void moveArm() {
        // Note: if disabled, the start call will automatically move the MP state to enabled

        proximalMotorRunning = true;
        proximalMotor.startMotionProfile(proximalBufferedStream, ArmConstants.minBufferedPoints, TalonFXControlMode.MotionProfile.toControlMode());

        distalMotorRunning = true;
        distalMotor.startMotionProfile(distalBufferedStream, ArmConstants.minBufferedPoints, TalonFXControlMode.MotionProfile.toControlMode());

        // start path timer
        pathStartedTime = Timer.getFPGATimestamp();
    }

    public void moveProximalArmHome() {
        proximalMotor.set(ControlMode.MotionMagic, ArmConstants.proximalHomePosition);
    }

    public void moveDistalArmHome() {
        distalMotor.set(ControlMode.MotionMagic, ArmConstants.distalHomePosition);
    }

    public void stopArm() {
        proximalMotor.set(TalonFXControlMode.MotionProfile, SetValueMotionProfile.Hold.value);
        distalMotor.set(TalonFXControlMode.MotionProfile, SetValueMotionProfile.Hold.value);
    }

    public boolean isArmMoving() {
        return proximalMotorRunning || distalMotorRunning;
    }

    private int getPathIndex() {
        int pointsLastIndex = currentPath.getNumberOfPoints()-1;
        double elapsedTimeMS = (Timer.getFPGATimestamp() - pathStartedTime) * 1000;
        int pointsProcessed = (int)(elapsedTimeMS / ArmConstants.pointDurationMS);
        int position = (currentDirection == Direction.FORWARD)? profileStartIndex + pointsProcessed : profileStartIndex - pointsProcessed;
        // make sure we don't end up with an array out of bounds exception
        if(position > pointsLastIndex) {
            return pointsLastIndex;
        } else if (position < 0) {
            return 0;
        }

        return position;
    }


    /*
     * METHODS FOR INITIALIZING THE Intake/WRIST
     */

    public void initializeWrist() {
        wristMotor.restoreFactoryDefaults();
        wristMotor.setSmartCurrentLimit(ArmConstants.WRIST_CURRENT_LIMIT);
        wristPIDController = wristMotor.getPIDController();
        wristEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

        // set PID coefficients
        wristPIDController.setP(ArmConstants.wristP);
        wristPIDController.setI(ArmConstants.wristI);
        wristPIDController.setD(ArmConstants.wristD);
        wristPIDController.setIZone(ArmConstants.wristIz);
        wristPIDController.setFF(ArmConstants.wristFF);
        wristPIDController.setOutputRange(ArmConstants.wristMinOutput, ArmConstants.wristMaxOutput);
        wristPIDController.setFeedbackDevice(wristEncoder);

        // set smart motion coefficients
        int smartMotionSlot = 0;
        wristPIDController.setSmartMotionMaxVelocity(ArmConstants.wristMaxVel, smartMotionSlot);
        wristPIDController.setSmartMotionMinOutputVelocity(ArmConstants.wristMinVel, smartMotionSlot);
        wristPIDController.setSmartMotionMaxAccel(ArmConstants.wristMaxAcc, smartMotionSlot);
        wristPIDController.setSmartMotionAllowedClosedLoopError(ArmConstants.wristAllowedErr, smartMotionSlot);
    }

    public void initializeIntake() {
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(ArmConstants.INTAKE_CURRENT_LIMIT_A);
        intakeMotor.setInverted(false);
        intakeMotor.setIdleMode(IdleMode.kBrake);
       // intakePIDController = intakeMotor.getPIDController();
      //  intakePIDController.setReference(ArmConstants.INTAKE_OUTPUT_POWER, CANSparkMax.ControlType.kVoltage);

    }

    public void moveWrist(double position, double maxVelocity) {
        int smartMotionSlot = 0;
        System.out.println("Moving wrist - velocity: " + maxVelocity + "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        //wristPIDController.setSmartMotionMaxVelocity(maxVelocity, smartMotionSlot);
        wristPIDController.setReference(position, CANSparkMax.ControlType.kSmartMotion);
    }

    public void moveWristHome() {
        moveWrist(ArmConstants.wristHomePosition, ArmConstants.wristMaxVel);
        wristFlexed = false;
    }

    public void stopWrist() {
        wristPIDController.setReference(0.0, CANSparkMax.ControlType.kVoltage);
    }


    public void intake() {
        intakeMotor.setSmartCurrentLimit(ArmConstants.INTAKE_CURRENT_LIMIT_A);
        intakeMotor.set(cone == true?1.0:-1.0);
        intaking = true;
        intakingTimer = Timer.getFPGATimestamp(); 
        System.out.println("Intaking");
    }

    public void eject() {
        ejecting = true;
        intakeMotor.setSmartCurrentLimit(ArmConstants.INTAKE_CURRENT_LIMIT_A);
        intakeMotor.set(cone==true?-1.0: 1.0);
        intaking = false;
        intakingTimer = null;
        System.out.println("Ejecting");
    }

    public void holdIntake() {
        intakeMotor.setSmartCurrentLimit(ArmConstants.INTAKE_HOLD_CURRENT_LIMIT_A);
        intakeMotor.set(cone==true?ArmConstants.INTAKE_HOLD_POWER: -1* ArmConstants.INTAKE_HOLD_POWER);
        intaking = false;
        intakingTimer = null;
        System.out.println("holding");
    }

    public void stopIntake() {
        intakeMotor.setSmartCurrentLimit(ArmConstants.INTAKE_CURRENT_LIMIT_A);
        intakeMotor.set(0);
        intaking = false;
        intakingTimer = null;
        System.out.println("Stopped");
    }


    /*-
     * PERIODIC LOGIC
     */

    @Override
    public void periodic() {
        boolean isArmMovingAtPeriodicStart = isArmMoving();

        if(stateMachine != null) {
            stateMachine.periodic(); // prompt the state machine to process any periodic tasks
        }

        if(proximalMotorRunning && proximalMotor.isMotionProfileFinished()) {
            // Note: when motion profile is finished it should be automatically set to HOLD state and will attempt to maintain final position
            proximalMotorRunning = false;
        }

        if(distalMotorRunning && distalMotor.isMotionProfileFinished()) {
            // Note: when motion profile is finished it should be automatically set to HOLD state and will attempt to maintain final position
            distalMotorRunning = false;
        }

        if(!wristFlexed && isArmMoving() && currentDirection == Direction.FORWARD) {
            int currentIndex = getPathIndex();
            int wristFlexIndex = currentPath.getWristFlexIndex();
            if(currentIndex >= wristFlexIndex) {
                // move the wrist into the flexed position for this path
                moveWrist(currentPath.getWristFlexPosition(), currentPath.getWristMaxVelocity());
                wristFlexed = true;
            }
        } else if(wristFlexed && isArmMoving() && currentDirection == Direction.REVERSE) {
            int currentIndex = getPathIndex();
            int wristExtendIndex = currentPath.getWristExtendIndex();
            if(currentIndex <= wristExtendIndex) {
                // move the wrist back into extended (home) position
                moveWrist(ArmConstants.wristHomePosition, currentPath.getWristMaxVelocity());
                wristFlexed = false;

                if(ejecting) {
                    stopIntake();
                }
            }
        }

        if(isArmMovingAtPeriodicStart && !isArmMoving()) { // arm was moving, but has now stopped
            if(currentDirection == Direction.REVERSE) { // we completed retracting
                currentPath = null;
                currentDirection = null;
                pathStartedTime = 0;
            } 
        }
        
        if ((intakingTimer != null) && ((Timer.getFPGATimestamp() - intakingTimer) > 0.5)  &&  (Math.abs(intakeMotor.getEncoder().getVelocity()) < 60)) {
            holdIntake();
            System.out.println("holdingig intake");

        }



        doSD();
    }


    /*
     * Helper methods for buffer handling and logging
     */
    
    private double armExtension() {
        return ArbitraryFeedForward.armExtension(proximalMotor.getSelectedSensorPosition(0), distalMotor.getSelectedSensorPosition(0));
    }
    
    private double distalArmExtension(){
        return ArbitraryFeedForward.distalArmExtension(distalMotor.getSelectedSensorPosition(0));
    }

    private double getArbitraryFeedForwardForProximalArm(){
        return ArbitraryFeedForward.getArbitraryFeedForwardForProximalArm(proximalMotor.getSelectedSensorPosition(0), distalMotor.getSelectedSensorPosition(0));
    }

    private double getArbitraryFeedForwardForDistalArm(){
        return ArbitraryFeedForward.getArbitraryFeedForwardForDistalArm(distalMotor.getSelectedSensorPosition(0));
    }
    
    public void doSD() {
        SmartDashboard.putNumber("DistalArm Absolute  ",distalAbsolute.getAverageValue());
        SmartDashboard.putNumber("ProximalArm Absolute  ",proximalAbsolute.getAverageValue());
        SmartDashboard.putNumber("DistalArm Relative  ",distalMotor.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("ProximalArm Relative  ",proximalMotor.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Proximal Motor percent", proximalMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Distal Motor Perennt Output", distalMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Davids Distall Calc", getArbitraryFeedForwardForDistalArm());
        SmartDashboard.putNumber("Davids Proximal Calc", getArbitraryFeedForwardForProximalArm());
        SmartDashboard.putNumber("Absolute wrist Encoder", wristEncoder.getPosition());
        SmartDashboard.putNumber("OutputCurrent", intakeMotor.getOutputCurrent());

    }

    public void resetArmEncoders() {
        proximalMotor.setSelectedSensorPosition((proximalAbsolute.getAverageValue() - ArmConstants.proximalAbsoluteTicsCenter) * ArmConstants.proximalRelativeTicsPerAbsoluteTick);
        distalMotor.setSelectedSensorPosition((distalAbsolute.getAverageValue()- ArmConstants.distalAbsoluteTicsCenter) * ArmConstants.distalRelativeTicsPerAbsoluteTick);
        System.out.println("setting distal to " + (distalAbsolute.getAverageValue()- ArmConstants.distalAbsoluteTicsCenter) * ArmConstants.distalRelativeTicsPerAbsoluteTick);
        System.out.println("setting proximal to " + (proximalAbsolute.getAverageValue() - ArmConstants.proximalAbsoluteTicsCenter) * ArmConstants.proximalRelativeTicsPerAbsoluteTick);
    }


    public void setArmMotors(double pAxis, double dAxis) {
        proximalMotor.set(ControlMode.PercentOutput, pAxis);
        distalMotor.set(ControlMode.PercentOutput, dAxis);
    }


    /*
     * Methods for recording arm movement
     */
    public void startRecordingArmPath() {
        System.out.println("\n ------------------- STARTED RECORDING ARM PATH ---------------------- \n");
        armPathLogger = LogWriter.getLogger(Log.ARM_PATH_RECORDING, ArmPathRecording.class);
    }

    public void stopRecordingArmPath() {
        System.out.println("\n ------------------- STOPPED RECORDING ARM PATH ---------------------- \n");
        armPathLogger.flush();
        armPathLogger = null;
    }

    public void writeArmPathValues() {
		/* Read the sensors */
		double proximal_pos = proximalMotor.getSelectedSensorPosition();
		double proximal_vel = proximalMotor.getSelectedSensorVelocity();
		double distal_pos = distalMotor.getSelectedSensorPosition();
		double distal_vel = distalMotor.getSelectedSensorVelocity();
		double wrist_pos = wristEncoder.getPosition();
		if(armPathLogger != null) {
			ArmPathRecording positions = new ArmPathRecording(
				proximal_pos, proximal_vel, distal_pos, distal_vel, wrist_pos);
			// write positions to csv file
			armPathLogger.add(positions);
		}
	}

    public boolean isArmRecordingRunning() {
        return (armPathLogger != null);
    }

    public boolean isCone() {
        return cone;
    }

    public void setCone(boolean b) {
        cone = b;
    }
}
