package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import frc.robot.state.arm.ArmStateMachine.MovementType;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.LogConstants;
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

    // arm recording
    Logger armPathLogger;
    
    // state tracking
    private ArmPath currentPath = null;
    private Direction currentDirection = null;
    private boolean proximalMPRunning = false;
    private boolean distalMPRunning = false;


    public ArmSubsystem() {
        System.out.println("ArmSubsystem: Starting up!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
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

    public void resetToHome() {
        System.out.println("ArmSubsystem: Resetting to home position!!!!!!!!!!!!!!!!!!!");
        moveWrist(ArmConstants.wristHomePosition, ArmConstants.wristMaxVel);
        proximalMotor.set(ControlMode.MotionMagic, ArmConstants.proximalHomePosition);
        distalMotor.set(ControlMode.MotionMagic, ArmConstants.distalHomePosition);
        stateMachine.completedArmMovement();
    }


    /*
     * PROXIMAL/DISTAL ARM MOTOR INIT
     */
    
    private void initializeArmMotors() {
        System.out.println("ArmSubsystem: Initializing arm motors!!!!!!!!!!!!!!!!!!!!!!!!!");
        proximalMotor = new WPI_TalonFX(ArmConstants.proximalCancoderId, "canivore1");
        initializeTalonMotor(proximalMotor, TalonFXInvertType.CounterClockwise);

        distalMotor = new WPI_TalonFX(ArmConstants.distalCancoderId, "canivore1");
        initializeTalonMotor(distalMotor, TalonFXInvertType.CounterClockwise);

        // make sure any previous motion profile trajectories are not present
        proximalMotor.clearMotionProfileTrajectories();
        distalMotor.clearMotionProfileTrajectories();

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
        motor.setNeutralMode(LogWriter.isArmRecordingEnabled()? NeutralMode.Coast : NeutralMode.Brake);
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
        System.out.println("Starting MotionProfile");
        proximalMotor.startMotionProfile(proximalBufferedStream, ArmConstants.minBufferedPoints, TalonFXControlMode.MotionProfile.toControlMode());
        System.out.println("proximal##################" + proximalMotor.isMotionProfileFinished());
        distalMPRunning = true;
        distalMotor.startMotionProfile(distalBufferedStream, ArmConstants.minBufferedPoints, TalonFXControlMode.MotionProfile.toControlMode());
        System.out.println("distal##################" + distalMotor.isMotionProfileFinished());
        stateMachine.startedPath();
    }

    public void adjustProximalArm(double targetPosition) {
        proximalMotor.set(ControlMode.MotionMagic, targetPosition);
    }

    public void adjustProximalArmVelocity(double velocity) {
        proximalMotor.set(ControlMode.Velocity, velocity);
    }

    public void adjustDistalArm(double targetPosition) {
        distalMotor.set(ControlMode.MotionMagic, targetPosition);
    }

    public void adjustDistalArmVelocity(double velocity) {
        distalMotor.set(ControlMode.Velocity, velocity);
    }

    public void stopArm() {
        proximalMotor.set(TalonFXControlMode.MotionProfile, SetValueMotionProfile.Hold.value);
        distalMotor.set(TalonFXControlMode.MotionProfile, SetValueMotionProfile.Hold.value);
    }

    public void allowArmManipulation() {
        stopIntake();
        proximalMotor.set(ControlMode.PercentOutput, 0);
        distalMotor.set(ControlMode.PercentOutput, 0);
        wristPIDController.setReference(0.0, CANSparkMax.ControlType.kVoltage);
    }

    public boolean isMotionProfileRunning() {
        return proximalMPRunning || distalMPRunning;
    }

    public double getProximalArmPosition() {
        return proximalMotor.getSelectedSensorPosition(0);
    }

    public double getDistalArmPosition() {
        return distalMotor.getSelectedSensorPosition(0);
    }


    /*
     * WRIST/INTAKE MOTOR INIT
     */

    public void initializeWrist() {
        wristMotor.restoreFactoryDefaults();
        wristMotor.setSmartCurrentLimit(ArmConstants.WRIST_CURRENT_LIMIT);
        wristMotor.setIdleMode(LogWriter.isArmRecordingEnabled()? IdleMode.kCoast : IdleMode.kBrake);
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
    }

    public double getWristPosition() {
        return wristEncoder.getPosition();
    }
    
    /*
     * INTAKE MOTOR MOVEMENT
     */

    public void intake() {
        intakeMotor.setSmartCurrentLimit(ArmConstants.INTAKE_CURRENT_LIMIT_A);
        double intakeSpeed = 0.8;
        if(stateMachine.getMovementType() == MovementType.PICKUP_DOWNED_CONE) {
            intakeSpeed = 0.8;
            System.out.println("ArmSubsystem: intaking DOWNED CONE, speed = " + intakeSpeed);
        } else if(stateMachine.getGamePiece() == GamePiece.CONE) {
            intakeSpeed = 0.8;
            System.out.println("ArmSubsystem: intaking CONE, speed = " + intakeSpeed);
        } else {
            intakeSpeed = -0.7;
            System.out.println("ArmSubsystem: intaking CUBE, speed = " + intakeSpeed);
        }
        intakeMotor.set(intakeSpeed);
    }

    public void eject() {
        intakeMotor.setSmartCurrentLimit(ArmConstants.EJECT_CURRENT_LIMIT);
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

    public boolean isIntakeAtStartedVelocity() {
        return (Math.abs(intakeMotor.getEncoder().getVelocity()) > ArmConstants.intakeStartedVelocityThreshold);
    }

    public boolean isIntakeAtHoldingVelocity() {
        return (Math.abs(intakeMotor.getEncoder().getVelocity()) < ArmConstants.intakeHoldingVelocityThreshold);
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
            if(currentDirection == Direction.REVERSE) { // we completed retracting
                currentPath = null;
                currentDirection = null;
                // specifically notify that this completion was a retraction
                stateMachine.completedArmRetraction();
            } else {
                stateMachine.completedArmMovement();
            } 
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

         if (LogConstants.loggingEnabled ) {
        SmartDashboard.putNumber("Proximal Trajectory Position", proximalMotor.getActiveTrajectoryPosition());
        SmartDashboard.putNumber("Proximal Trajectory Velocity", proximalMotor.getActiveTrajectoryVelocity());
        SmartDashboard.putNumber("Proximal Motor Percent", proximalMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Proximal Actual Position", proximalMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Proximal Actual Velocity", proximalMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Proximal Closed Loop Error", proximalMotor.getClosedLoopError());
        SmartDashboard.putNumber("Proximal Closed Loop Target", proximalMotor.getClosedLoopTarget());
        SmartDashboard.putNumber("Proximal Arb Feed Forward", proximalMotor.getActiveTrajectoryArbFeedFwd());

        SmartDashboard.putNumber("Distal Trajectory Position", distalMotor.getActiveTrajectoryPosition());
        SmartDashboard.putNumber("Distal Trajectory Velocity", distalMotor.getActiveTrajectoryVelocity());
        SmartDashboard.putNumber("Distal Motor Percent", distalMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Distal Actual Position", distalMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Distal Actual Velocity", distalMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Distal Closed Loop Error", distalMotor.getClosedLoopError());
        SmartDashboard.putNumber("Distal Closed Loop Target", distalMotor.getClosedLoopTarget());
        SmartDashboard.putNumber("Distal Arb Feed Forward", distalMotor.getActiveTrajectoryArbFeedFwd());

    }

    }


    /*
    * METHODS FOR CHECKING/SETTING ENCODER VALUES
    * Note: much of the code below is oriented toward working around spotty absolute encoder values
    * this behavior may not be a desirable approach if we resolve that issue
    */

    // This is the reset called outside of auto, however in this state, if we don't have reasonable absolute values
    // we must discard them as we cannot estimate where we are like we can in auto where we start from a consistent position
    public void resetArmEncoders() {
        // if we are getting reasonable values use them, otherwise do not set anything
        // note: we provide wider out of bounds thresholds for this check b/c we don't know where we are
        if(!isProximalAbsoluteEncoderOutOfBounds(ArmConstants.proximalAbsoluteBounds)) {
            resetProximalEncoder(proximalAbsolute.getAverageValue());
        }
        if(!isDistalAbsoluteEncoderOutOfBounds(ArmConstants.distalAbsoluteBounds)) {
            resetDistalEncoder(distalAbsolute.getAverageValue());
        }
    }

    // This is the reset called from autoInit, we are starting from the same position in auto every time which allows us
    // to reasonably estimate the absolute values in the instance that we are not getting reliable values from the potentiometer
    public void resetArmEncodersForAuto() {
        // if we are getting reasonable values use them, otherwise provide estimated proximal/distal absolutes
        // note: we provide narrower out of bounds thresholds for this check b/c we know about where we are
        int proximalAbsoluteVal = isProximalAbsoluteEncoderOutOfBounds(ArmConstants.proximalAbsoluteBoundsAuto)? ArmConstants.proximalEstimatedAutoAbsolute : proximalAbsolute.getAverageValue();
        resetProximalEncoder(proximalAbsoluteVal);
        int distalAbsoluteVal = isDistalAbsoluteEncoderOutOfBounds(ArmConstants.distalAbsoluteBoundsAuto)? ArmConstants.distalEstimatedAutoAbsolute : distalAbsolute.getAverageValue();
        resetDistalEncoder(distalAbsoluteVal);
    }

    public void resetProximalEncoder(int proximalAbsoluteVal) {
        proximalMotor.setSelectedSensorPosition((proximalAbsoluteVal - ArmConstants.proximalAbsoluteTicsCenter) * ArmConstants.proximalRelativeTicsPerAbsoluteTick);
        System.out.println("ArmSubsystem: setting proximal to " + (proximalAbsoluteVal - ArmConstants.proximalAbsoluteTicsCenter) * ArmConstants.proximalRelativeTicsPerAbsoluteTick);
    }

    public void resetDistalEncoder(int distalAbsoluteVal) {
        distalMotor.setSelectedSensorPosition((distalAbsoluteVal - ArmConstants.distalAbsoluteTicsCenter) * ArmConstants.distalRelativeTicsPerAbsoluteTick);
        System.out.println("ArmSubsystem: setting distal to " + (distalAbsoluteVal - ArmConstants.distalAbsoluteTicsCenter) * ArmConstants.distalRelativeTicsPerAbsoluteTick);
    }

    // this helper method is used during disabled to notify the team if the absolute values seems out of whack
    // this enables them to move the arm around a bit to see if they can get better readings
    public boolean isInEncodersOutOfBoundsCondition() {
        if(isProximalAbsoluteEncoderOutOfBounds(ArmConstants.proximalAbsoluteBounds) || isDistalAbsoluteEncoderOutOfBounds(ArmConstants.distalAbsoluteBounds)) {
          System.out.println("ArmSubsystem: WARNING reporting out of bounds condition with absolute encoders");
          return true;
        }
        return false;
    }

    // check for unreasonable (out-of-bounds) conditions using the  upper/lower bounds provided
    private boolean isProximalAbsoluteEncoderOutOfBounds(int[] range) {
        int proximalAbsoluteVal = proximalAbsolute.getAverageValue();
        if(proximalAbsoluteVal < range[0] || proximalAbsoluteVal > range[1]) {
            System.out.println("ArmSubsystem: WARNING proximal absolute encoder reading out of bounds: " + proximalAbsoluteVal);
            return true;
        }
        return false;
    }

    // check for unreasonable (out-of-bounds) conditions using the  upper/lower bounds provided
    private boolean isDistalAbsoluteEncoderOutOfBounds(int[] range) {
        int distalAbsoluteVal = distalAbsolute.getAverageValue();
        if(distalAbsoluteVal < range[0] || distalAbsoluteVal > range[1]) {
            System.out.println("ArmSubsystem: WARNING distal absolute encoder reading out of bounds: " + distalAbsoluteVal);
            return true;
        }
        return false;
    }


    /*
     * Used only for testing the arm in percent output
     */

    public void testArmMotors(double pAxis, double dAxis) {
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
