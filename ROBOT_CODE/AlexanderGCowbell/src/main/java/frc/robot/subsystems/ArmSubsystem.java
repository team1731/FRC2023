package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motion.*;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.state.arm.ArmStateMachine;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GamePiece;
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
    private AbsoluteEncoder wristEncoder;
    private boolean wristResetting = false;

    // arm recording
    Logger armPathLogger;
    
    // state tracking
    private ArmPath currentPath = null;
    private Direction currentDirection = null;
    private boolean proximalMPRunning = false;
    private boolean distalMPRunning = false;
    private boolean proximalArmResetting = false;
    private boolean distalArmResetting = false;


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

    public Direction getDirection() {
        return currentDirection;
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
            stateMachine.resetState();
        }        
    }


    /*
     * PROXIMAL/DISTAL ARM MOTOR INIT
     */
    
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
        motor.configAllSettings(talonConfig);

        // Note: these two lines required for motion magic to work
        motor.configMotionCruiseVelocity(15000, 30);
		motor.configMotionAcceleration(6000, 30);

        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
        motor.configNominalOutputForward(0, 30);
		motor.configNominalOutputReverse(0, 30);
		motor.configPeakOutputForward(0.5, 30);
		motor.configPeakOutputReverse(-0.5, 30);
        motor.setInverted(invertType);
    }


    /*
     * PROXIMAL/DISTAL ARM MOTOR MOVEMENT
     */

    public void startArmMovement(ArmPath armPath) {
        currentPath = armPath;
        currentDirection = Direction.FORWARD;
        initArmMovement(0);
    }

    public void reverseArmMovment(int startIndex) {
        currentDirection = Direction.REVERSE;
        initArmMovement(startIndex);
    }

    private void initArmMovement(int startIndex) {
        // init motion profile buffers for proximal/distal motors
        proximalBufferedStream = currentPath.getInitializedBuffer(ArmMotor.PROXIMAL, startIndex, currentDirection);
        distalBufferedStream = currentPath.getInitializedBuffer(ArmMotor.DISTAL, startIndex, currentDirection);

        // start arm movement
        moveArm();
    }

    private void moveArm() {
        // Note: if disabled, the start call will automatically move the MP state to enabled

        proximalMPRunning = true;
        proximalMotor.startMotionProfile(proximalBufferedStream, ArmConstants.minBufferedPoints, TalonFXControlMode.MotionProfile.toControlMode());

        distalMPRunning = true;
        distalMotor.startMotionProfile(distalBufferedStream, ArmConstants.minBufferedPoints, TalonFXControlMode.MotionProfile.toControlMode());

        stateMachine.startedPath();
    }

    public void moveProximalArmHome() {
        proximalMotor.set(ControlMode.MotionMagic, ArmConstants.proximalHomePosition);
        proximalArmResetting = true;
    }

    public void moveDistalArmHome() {
        distalMotor.set(ControlMode.MotionMagic, ArmConstants.distalHomePosition);
        distalArmResetting = true;
    }

    public void stopArm() {
        proximalMotor.set(TalonFXControlMode.MotionProfile, SetValueMotionProfile.Hold.value);
        distalMotor.set(TalonFXControlMode.MotionProfile, SetValueMotionProfile.Hold.value);
    }

    public boolean isMotionProfileRunning() {
        return proximalMPRunning || distalMPRunning;
    }


    /*
     * WRIST/INTAKE MOTOR INIT
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
    }


    /*
     * WRIST MOTOR MOVEMENT
     */

    public void moveWrist(double position, double maxVelocity) {
        int smartMotionSlot = 0;
        wristPIDController.setSmartMotionMaxVelocity(maxVelocity, smartMotionSlot);
        wristPIDController.setReference(position, CANSparkMax.ControlType.kSmartMotion);
    }

    public void moveWristHome() {
        moveWrist(ArmConstants.wristHomePosition, ArmConstants.wristMaxVel);
        wristResetting = true;
    }

    public void stopWrist() {
        wristPIDController.setReference(0.0, CANSparkMax.ControlType.kVoltage);
    }


    /*
     * INTAKE MOTOR MOVEMENT
     */

    public void intake() {
        intakeMotor.setSmartCurrentLimit(ArmConstants.INTAKE_CURRENT_LIMIT_A);
        intakeMotor.set((stateMachine.getGamePiece() == GamePiece.CONE)? 1.0 : -1.0);
    }

    public void eject() {
        intakeMotor.setSmartCurrentLimit(ArmConstants.INTAKE_CURRENT_LIMIT_A);
        intakeMotor.set((stateMachine.getGamePiece() == GamePiece.CONE)? -1.0 : 1.0);
    }

    public void holdIntake() {
        intakeMotor.setSmartCurrentLimit(ArmConstants.INTAKE_HOLD_CURRENT_LIMIT_A);
        intakeMotor.set((stateMachine.getGamePiece() == GamePiece.CONE)? ArmConstants.INTAKE_HOLD_POWER : -1 * ArmConstants.INTAKE_HOLD_POWER);
    }

    public void stopIntake() {
        intakeMotor.setSmartCurrentLimit(ArmConstants.INTAKE_CURRENT_LIMIT_A);
        intakeMotor.set(0);
    }

    public boolean isIntakeAtHoldingVelocity() {
        return (Math.abs(intakeMotor.getEncoder().getVelocity()) < 60);
    }

    
    /*-
     * PERIODIC LOGIC
     */

    @Override
    public void periodic() {
        boolean isMPRunningAtPeriodicStart = isMotionProfileRunning();

        if(stateMachine != null) {
            stateMachine.periodic(); // prompt the state machine to process any periodic tasks
        }

        if(proximalMPRunning && proximalMotor.isMotionProfileFinished()) {
            // Note: when motion profile is finished it should be automatically set to HOLD state and will attempt to maintain final position
            proximalMPRunning = false;
        }

        if(distalMPRunning && distalMotor.isMotionProfileFinished()) {
            // Note: when motion profile is finished it should be automatically set to HOLD state and will attempt to maintain final position
            distalMPRunning = false;
        }

        if(isMPRunningAtPeriodicStart && !isMotionProfileRunning()) { // arm was moving, but has now stopped
            stateMachine.completedArmMovement();
            if(currentDirection == Direction.REVERSE) { // we completed retracting
                currentPath = null;
                currentDirection = null;
            } 
        }

        if(wristResetting && Math.abs(ArmConstants.wristHomePosition - wristMotor.getEncoder().getPosition()) < ArmConstants.wristResetPostionThreshold) {
            wristResetting = false;
            stateMachine.completedArmMovement();
        }

        if(proximalArmResetting && Math.abs(proximalMotor.getActiveTrajectoryVelocity()) < 60) {
            proximalArmResetting = false;
            stateMachine.completedArmMovement();
        }

        if(distalArmResetting && Math.abs(distalMotor.getActiveTrajectoryVelocity()) < 60) {
            distalArmResetting = false;
            stateMachine.completedArmMovement();
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
}
