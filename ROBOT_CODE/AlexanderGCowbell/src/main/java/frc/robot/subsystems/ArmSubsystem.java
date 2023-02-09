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
import frc.robot.state.arm.ArmInput;
import frc.robot.state.*;
import frc.robot.Constants.StateConstants.ResultCode;
import frc.robot.Constants.ArmConstants;
import frc.data.mp.*;
import frc.data.mp.ArmPath.ArmMotor;
import frc.data.mp.ArmPath.Direction;

public class ArmSubsystem extends SubsystemBase implements StateHandler {
    private StateMachine stateMachine;

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

    
    // state tracking
    private ArmPath currentPath = null;
    private Direction currentDirection = null;
    private double pathStartedTime = 0;
    private boolean proximalMotorRunning = false;
    private boolean distalMotorRunning = false;



    public ArmSubsystem() {
        initializeArmMotors();
        distalAbsolute = new AnalogInput(0);
        proximalAbsolute = new AnalogInput(1);
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
        wristPIDController.setReference(0.63, CANSparkMax.ControlType.kSmartMotion);
        distalMotor.set(ControlMode.MotionMagic,1700);
        proximalMotor.set(ControlMode.MotionMagic, 1500);
    }


    private void initializeTalonMotor(WPI_TalonFX motor, TalonFXInvertType invertType) {
        TalonFXConfiguration talonConfig = new TalonFXConfiguration(); // factory default settings
        talonConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        talonConfig.neutralDeadband = ArmConstants.kNeutralDeadband; /* 0.1 % super small for best low-speed control */
        talonConfig.slot0.kF = ArmConstants.kGains_MotProf.kF;
        talonConfig.slot0.kP = ArmConstants.kGains_MotProf.kP;
        talonConfig.slot0.kI = ArmConstants.kGains_MotProf.kI;
        talonConfig.slot0.kD = ArmConstants.kGains_MotProf.kD;
        talonConfig.slot0.integralZone = (int) ArmConstants.kGains_MotProf.kIzone;
        talonConfig.slot0.closedLoopPeakOutput = ArmConstants.kGains_MotProf.kPeakOutput;
        motor.configMotionCruiseVelocity(15000, 30);
		motor.configMotionAcceleration(6000, 30);
        // talonConfig.slot0.allowableClosedloopError // left default for this example
        // talonConfig.slot0.maxIntegralAccumulator; // left default for this example
        // talonConfig.slot0.closedLoopPeriod; // left default for this example
        motor.configAllSettings(talonConfig);
        motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 30, 0.2));
        motor.setInverted(invertType);
    }

        // home for any logic needed to reset, especially when robot moves to disabled state
    // ensures the motors are stopped and motion profiles are cleared/disabled, 
    // so motor doesn't try to process last profile when re-enabled
    public void reset() {
        // ensure motors are stopped
        proximalMotor.set(TalonFXControlMode.PercentOutput, 0);
        distalMotor.set(TalonFXControlMode.PercentOutput, 0);

        // reset to disabled state w/ no motion profiles
        proximalMotor.clearMotionProfileTrajectories();
        proximalMotor.set(TalonFXControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
        distalMotor.clearMotionProfileTrajectories();
        distalMotor.set(TalonFXControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
    }

    public void startArmMovement(ArmPath armPath) {
        if(currentPath != null) {
            notifyStateMachine(ResultCode.INVALID_REQUEST, "Invalid request: cannot initialize new movement until current path is complete");
            return;
        }

        // init motion profile buffers for proximal/distal motors
        currentDirection = Direction.FORWARD;
        proximalBufferedStream = armPath.getInitializedBuffer(ArmMotor.PROXIMAL, 0, currentDirection);
        distalBufferedStream = armPath.getInitializedBuffer(ArmMotor.DISTAL, 0, currentDirection);

        notifyStateMachine(ResultCode.SUCCESS, "Successfully initialized arm profile buffers");

        // start arm movement
        pathStartedTime = Timer.getFPGATimestamp();
        moveArm();
    }

    public void reverseArmMovment() {
        if(currentPath == null || currentDirection == Direction.REVERSE) {
            notifyStateMachine(ResultCode.INVALID_REQUEST, "Invalid request: cannot be reversed, either retracted or already reversing");
            return;
        }

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
            double elapsedTimeMS = (Timer.getFPGATimestamp() - pathStartedTime) * 1000;
            startPosition = (int)(elapsedTimeMS / 10);
            // make sure we don't end up in array out of bounds exception
            startPosition = startPosition > pointsLastIndex? pointsLastIndex : startPosition;
        }

        currentDirection = Direction.REVERSE;
        proximalBufferedStream = currentPath.getInitializedBuffer(ArmMotor.PROXIMAL, startPosition, currentDirection);
        distalBufferedStream = currentPath.getInitializedBuffer(ArmMotor.DISTAL, startPosition, currentDirection);

        // restart arm movement
        moveArm();
    }

    public void moveArm() {
        if(isArmMoving()) {
            notifyStateMachine(ResultCode.INVALID_REQUEST, "Invalid request: attempt to start new movement when arm is already moving");
            return;
        }

        // start motion profile for proximal/distal motors
        ErrorCode code;

        proximalMotorRunning = true;
        // Note: if disabled, the start call will automatically move the MP state to enabled
        code = proximalMotor.startMotionProfile(proximalBufferedStream, 10, TalonFXControlMode.MotionProfile.toControlMode());
        if(code != ErrorCode.OK) {
            notifyStateMachine(ResultCode.ARM_MOTION_START_FAILED, "Failed to start proximal motion profile");
            return;
        }

        distalMotorRunning = true;
        // Note: if disabled, the start call will automatically move the MP state to enabled
        code = distalMotor.startMotionProfile(distalBufferedStream, 10, TalonFXControlMode.MotionProfile.toControlMode());
        if(code != ErrorCode.OK) {
            notifyStateMachine(ResultCode.ARM_MOTION_START_FAILED, "Failed to start distal motion profile");
            return;
        }

        notifyStateMachine(ResultCode.SUCCESS, "Successfully started arm movement");
    }

    public void stopArm() {
        proximalMotor.set(TalonFXControlMode.MotionProfile, SetValueMotionProfile.Hold.value);
        distalMotor.set(TalonFXControlMode.MotionProfile, SetValueMotionProfile.Hold.value);
    }

    public boolean isArmMoving() {
        return proximalMotorRunning || distalMotorRunning;
    }

    public boolean completedArmPathForward() {
        return pathStartedTime > 0 && !isArmMoving();
    }

    /*
     * METHODS FOR INITIALIZING THE Intake/WRIST
     */


    public void initializeWrist() {
        wristMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(30);
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
        // initialize motors
    //    intakeMotor = new CANSparkMax(deviceID, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(30);
        intakeMotor.setInverted(false);
        intakeMotor.setIdleMode(IdleMode.kBrake);
    

 
    }

    public void moveWrist(double rotations) {
        // TODO handle errors
    
        wristPIDController.setReference(rotations, CANSparkMax.ControlType.kSmartMotion);
        
    }

    public void stopWrist() {
        // TODO handle errors
    
        wristPIDController.setReference(0.0, CANSparkMax.ControlType.kVoltage);
        
    }


    public void intake() {
        intakeMotor.set(1.0);
    }

    public void eject() {
        intakeMotor.set(-1.0);
    }

    public void stopIntake() {
        intakeMotor.set (0);
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
        } else if(proximalMotorRunning) {
            logMotionProfileStatus(proximalMotor);
        }

        if(distalMotorRunning && distalMotor.isMotionProfileFinished()) {
            // Note: when motion profile is finished it should be automatically set to HOLD state and will attempt to maintain final position
            distalMotorRunning = false;
        } else if(distalMotorRunning) {
            logMotionProfileStatus(distalMotor);
        }

        if(isArmMovingAtPeriodicStart && !isArmMoving()) { // arm was moving, but has now stopped
            notifyStateMachine(ResultCode.SUCCESS, "Completed arm movement");

            if(currentDirection == Direction.REVERSE) { // we completed retracting
                currentPath = null;
                currentDirection = null;
                pathStartedTime = 0;
            } 
        }
        doSD();
    }


    /*
     * STATE HANDLER INTERFACE
     */

    public void registerStateMachine(StateMachine stateMachine) {
        this.stateMachine = stateMachine;
    }

    public void changeState(Input input, Object data) {
        ArmInput ai = (ArmInput)input;
        switch(ai) {
            case EXTEND_INIT:
            case RETRACT_INIT:
                //initializeArmMovement((MotionProfile[])data);
                break;
            case EXTEND_MOVE:
            case RETRACT_MOVE:
                //moveArm();
                break;
            case INTAKE:
            case RELEASE:
                // TODO implement, for the moment just send back success to keep the state process moving
                notifyStateMachine(ResultCode.SUCCESS, "TEST: sending success code for unimplemented step");
                break;
            default:
                // unrecognized state change, send back failure
                notifyStateMachine(ResultCode.INVALID_REQUEST, "Invalid request: state change request was not recognized");
        }
    }

    public void interruptStateChange() {
        // TODO implement
    }

    private void notifyStateMachine(ResultCode resultCode, String resultMessage) {
        if(stateMachine != null) {
            ArmInput input = resultCode == ResultCode.SUCCESS? ArmInput.SUCCESS : ArmInput.FAILED;
            StateChangeResult result = new StateChangeResult(resultCode, resultMessage, Timer.getFPGATimestamp());
            stateMachine.transition(input, result);
        }
    }


    /*
     * Helper methods for buffer handling and logging
     */

    private void logMotionProfileStatus(TalonFX motor) {
        // TODO implement logging, do so at reasonable intervals
        MotionProfileStatus status = new MotionProfileStatus();
        motor.getMotionProfileStatus(status);

        // pos = motor.getActiveTrajectoryPosition();
        // vel = motor.getActiveTrajectoryVelocity();
        // status.topBufferRem
        // status.topBufferCnt
        // status.btmBufferCnt
        // status.hasUnderrun
        // status.isUnderrun
        // status.activePointValid
        // status.isLast
        // status.profileSlotSelect
        // status.profileSlotSelect1
        // status.outputEnable.toString()
        // status.timeDurMs
    } 

    private double armExtension(double proximalTicks, double distalTicks){
        //Calculates how far the arm is extended using trigonometry and angles derived from data from motors 
        double distalDistance = (ArmConstants.distalArmLength * java.lang.Math.cos(3)) * java.lang.Math.sin(distalTicks * ArmConstants.distalTicksPerDegree); 
        double proximalDistance = ArmConstants.proximalArmLength * java.lang.Math.sin(proximalTicks * ArmConstants.proximalTicksPerDegree);
        return distalDistance + proximalDistance; 
    }
    private double armExtension(){
        return armExtension(proximalMotor.getSelectedSensorPosition(0), distalMotor.getSelectedSensorPosition(0));
    }
    private double distalArmExtension(double distalTicks){
        //Calculates how far the arm is extended using trigonometry and angles derived from data from motors 
        double distalDistance = (ArmConstants.distalArmLength * java.lang.Math.cos(3)) * java.lang.Math.sin(distalTicks * ArmConstants.distalTicksPerDegree); 
        return distalDistance; 
    }
    private double distalArmExtension(){
        return distalArmExtension(distalMotor.getSelectedSensorPosition(0));
    }
    private double getArbitraryFeedForwardForProximalArm(double proximalTicks, double distalTicks){
        return ArmConstants.ThrottleAtFullExtensionDistalAndProximal * (armExtension(proximalTicks, distalTicks) / ArmConstants.armFullExtensionDistance);
    }
    private double getArbitraryFeedForwardForProximalArm(){
        return getArbitraryFeedForwardForProximalArm(proximalMotor.getSelectedSensorPosition(0), distalMotor.getSelectedSensorPosition(0));
    }
    private double getArbitraryFeedForwardForDistalArm(double distalTicks){
        return ArmConstants.ThrottleAtFullExtensionDistal * (distalArmExtension(distalTicks) / ArmConstants.distalArmLength);
    }
    private double getArbitraryFeedForwardForDistalArm(){
        return getArbitraryFeedForwardForDistalArm(distalMotor.getSelectedSensorPosition(0));
    }
    
    public void doSD() {
        SmartDashboard.putNumber("DistalArm Absolute  ",distalAbsolute.getAverageValue());
        SmartDashboard.putNumber("ProximalArm Absolute  ",proximalAbsolute.getAverageValue());
        SmartDashboard.putNumber("DistalArm Relative  ",distalMotor.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("ProximalArm Relative  ",proximalMotor.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Proximal Motor percent", proximalMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Distal Motor Perennt Output", distalMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Davids Distall Calc", getArbitraryFeedForwardForDistalArm());
        SmartDashboard.putNumber("Ddavids Proximal Calc", getArbitraryFeedForwardForProximalArm());
        SmartDashboard.putNumber("Absolute wrist Encoder", wristEncoder.getPosition());

    }

    public void resetArmEncoders() {
        proximalMotor.setSelectedSensorPosition((proximalAbsolute.getAverageValue()- ArmConstants.proximalAbsoluteTicsCenter) * ArmConstants.proximalRelativeTicsPerAbsoluteTick);
        distalMotor.setSelectedSensorPosition((distalAbsolute.getAverageValue()- ArmConstants.distalAbsoluteTicsCenter) * ArmConstants.distalRelativeTicsPerAbsoluteTick);

    }


    public void setArmMotors(double pAxis, double dAxis) {
        proximalMotor.set(ControlMode.PercentOutput, pAxis);
        distalMotor.set(ControlMode.PercentOutput, dAxis);
    }
}
