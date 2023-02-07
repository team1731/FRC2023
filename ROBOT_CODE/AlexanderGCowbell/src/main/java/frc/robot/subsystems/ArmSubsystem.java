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

public class ArmSubsystem extends SubsystemBase implements StateHandler {
    private StateMachine stateMachine;

    // motors for the arm
    private WPI_TalonFX proximalMotor;
    private WPI_TalonFX distalMotor;
    private TalonFXConfiguration proximalTalonConfig;
    private TalonFXConfiguration distalTalonConfig;
    private AnalogInput distalAbsolute;
    private AnalogInput proximalAbsolute;


    // motion profiles/buffers for arm proximal and distal motors
    private BufferedTrajectoryPointStream proximalBufferedStream;
    private BufferedTrajectoryPointStream distalBufferedStream;

    // motors for the wrist/intake
    private static final int deviceID = 1;
    private CANSparkMax wristMotor;
    private CANSparkMax intakeMotor;
    private SparkMaxPIDController wristPIDController;
  
    private AbsoluteEncoder wristEncoder;

    
    // state tracking
    private boolean proximalMotorRunning = false;
    private boolean distalMotorRunning = false;



    public ArmSubsystem() {
        // setup motor and motion profiling members
        proximalMotor = new WPI_TalonFX(15, "canivore1");
      //  proximalMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 30, 0.2));
        distalMotor = new WPI_TalonFX(16, "canivore1");
      //  distalMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 30, 0.2));
        proximalTalonConfig = new TalonFXConfiguration(); // factory default settings
        distalTalonConfig = new TalonFXConfiguration(); // factory default settings
        proximalBufferedStream = new BufferedTrajectoryPointStream();
        distalBufferedStream = new BufferedTrajectoryPointStream();



        // setup wrist/hand motor members
        wristMotor = new CANSparkMax(ArmConstants.wristCancoderId, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(ArmConstants.intakeCancoderId, MotorType.kBrushless);

        // kick off initializations
        initializeProximalMotor();
        initializeDistalMotor();
        initializeWrist();
        initializeIntake();
        distalAbsolute = new AnalogInput(0);
        proximalAbsolute = new AnalogInput(1);
    }

    /*
     * METHODS FOR INITIALIZING AND MOVING THE ARM
     */

    // configures the arm motors, should be called 
    private void initializeProximalMotor() {
        // TODO clean up configuration code
        proximalTalonConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        proximalTalonConfig.neutralDeadband = ArmConstants.kNeutralDeadband; /* 0.1 % super small for best low-speed control */
        proximalTalonConfig.slot0.kF = ArmConstants.kGains_MotProf.kF;
        proximalTalonConfig.slot0.kP = ArmConstants.kGains_MotProf.kP;
        proximalTalonConfig.slot0.kI = ArmConstants.kGains_MotProf.kI;
        proximalTalonConfig.slot0.kD = ArmConstants.kGains_MotProf.kD;
        proximalTalonConfig.slot0.integralZone = (int) ArmConstants.kGains_MotProf.kIzone;
        proximalTalonConfig.slot0.closedLoopPeakOutput = ArmConstants.kGains_MotProf.kPeakOutput;
        // proximalTalonConfig.slot0.allowableClosedloopError // left default for this example
        // proximalTalonConfig.slot0.maxIntegralAccumulator; // left default for this example
        // proximalTalonConfig.slot0.closedLoopPeriod; // left default for this example
        proximalMotor.configAllSettings(proximalTalonConfig);
        //proximalMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 10.0, 10.0, 0.1));
        

        // pick the sensor phase and desired direction
        proximalMotor.setInverted(TalonFXInvertType.CounterClockwise);
    }

    private void initializeDistalMotor() {
        // TODO clean up configuration code
        distalTalonConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        distalTalonConfig.neutralDeadband = ArmConstants.kNeutralDeadband; /* 0.1 % super small for best low-speed control */
        distalTalonConfig.slot0.kF = ArmConstants.kGains_MotProf.kF;
        distalTalonConfig.slot0.kP = ArmConstants.kGains_MotProf.kP;
        distalTalonConfig.slot0.kI = ArmConstants.kGains_MotProf.kI;
        distalTalonConfig.slot0.kD = ArmConstants.kGains_MotProf.kD;
        distalTalonConfig.slot0.integralZone = (int) ArmConstants.kGains_MotProf.kIzone;
        distalTalonConfig.slot0.closedLoopPeakOutput = ArmConstants.kGains_MotProf.kPeakOutput;
        // proximalTalonConfig.slot0.allowableClosedloopError // left default for this example
        // proximalTalonConfig.slot0.maxIntegralAccumulator; // left default for this example
        // proximalTalonConfig.slot0.closedLoopPeriod; // left default for this example
        distalMotor.configAllSettings(distalTalonConfig);
        //distalMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 10.0, 10.0, 0.1));

        // pick the sensor phase and desired direction
        distalMotor.setInverted(TalonFXInvertType.CounterClockwise);
    }

    public void initializeArmMovement(MotionProfile[] profiles) {
        if(isArmMoving()) {
            notifyStateMachine(ResultCode.INVALID_REQUEST, "Invalid request: attempt to initialize when arm is already moving");
            return;
        }

        // init motion profile buffers for proximal/distal motors

        String errorMessage;
        errorMessage = initBuffer(profiles[0], proximalBufferedStream);
        if(errorMessage != null) {
            notifyStateMachine(ResultCode.ARM_BUFFERING_FAILED, "Proximal buffer failed to initialize with message: " + errorMessage);
            return;
        }
        errorMessage = initBuffer(profiles[1], distalBufferedStream);
        if(errorMessage != null) {
            notifyStateMachine(ResultCode.ARM_BUFFERING_FAILED, "Distal buffer failed to initialize with message: " + errorMessage);
            return;
        }

        notifyStateMachine(ResultCode.SUCCESS, "Successfully initialized arm profile buffers");
    }

    public void moveArm() {
        if(isArmMoving()) {
            notifyStateMachine(ResultCode.INVALID_REQUEST, "Invalid request: attempt to start new movement when arm is already moving");
            return;
        }

        // start motion profile for proximal/distal motors

        ErrorCode code;
        proximalMotorRunning = true;
        code = proximalMotor.startMotionProfile(proximalBufferedStream, 10, TalonFXControlMode.MotionProfile.toControlMode());
        if(code != ErrorCode.OK) {
            notifyStateMachine(ResultCode.ARM_MOTION_START_FAILED, "Failed to start proximal motion profile");
            return;
        }

        distalMotorRunning = true;
        code = distalMotor.startMotionProfile(distalBufferedStream, 10, TalonFXControlMode.MotionProfile.toControlMode());
        if(code != ErrorCode.OK) {
            notifyStateMachine(ResultCode.ARM_MOTION_START_FAILED, "Failed to start distal motion profile");
            return;
        }

        notifyStateMachine(ResultCode.SUCCESS, "Successfully started arm movement");
    }

    public boolean isArmMoving() {
        return proximalMotorRunning || distalMotorRunning;
    }

    public void resetSensorPosition() {
        proximalMotor.setSelectedSensorPosition(0);
        distalMotor.setSelectedSensorPosition(0);
    }

    /*
     * METHODS FOR INITIALIZING THE Intake/WRIST
     */
    public void initializeWrist() {
        // initialize motors
        wristMotor = new CANSparkMax(deviceID, MotorType.kBrushless);
        wristMotor.restoreFactoryDefaults();

        // initialze PID controller and encoder objects
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
        intakeMotor = new CANSparkMax(deviceID, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setInverted(false);
        intakeMotor.setIdleMode(IdleMode.kBrake);
    

 
    }

    public void moveWrist(double rotations) {
        // TODO handle errors
    
        wristPIDController.setReference(rotations, CANSparkMax.ControlType.kSmartMotion);
        System.out.println("Wrist position " + wristEncoder.getPosition());
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
            double position = proximalMotor.getSelectedSensorPosition(0) / 2048;
            System.out.println("Proximal position: " + proximalMotor.getSelectedSensorPosition(0) + "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            //proximalMotor.set(TalonFXControlMode.Position.toControlMode(), position);
            proximalMotorRunning = false;
        } else if(proximalMotorRunning) {
            logMotionProfileStatus(proximalMotor);
        }

        if(distalMotorRunning && distalMotor.isMotionProfileFinished()) {
            double position = distalMotor.getSelectedSensorPosition(0) / 2048;
            System.out.println("Distal position: " + distalMotor.getSelectedSensorPosition(0) + "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            //distalMotor.set(TalonFXControlMode.Position.toControlMode(), position);
            distalMotorRunning = false;
        } else if(distalMotorRunning) {
            logMotionProfileStatus(distalMotor);
        }

        if(isArmMoving()) { // arm is still moving
            // TODO implement, check for issues?
        }

        if(isArmMovingAtPeriodicStart && !isArmMoving()) { // arm was moving, but has now stopped
            // TODO any checking to ensure we are where we expect to be?
            notifyStateMachine(ResultCode.SUCCESS, "Completed arm movement");
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
                initializeArmMovement((MotionProfile[])data);
                break;
            case EXTEND_MOVE:
            case RETRACT_MOVE:
                moveArm();
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

    private String initBuffer(MotionProfile profile, BufferedTrajectoryPointStream bufferedStream) {
        int numberOfPoints = profile.numberOfPoints;
        double[][] points = profile.points;
        TrajectoryPoint point = new TrajectoryPoint(); 
        ErrorCode code;

        // clear the buffer, it may have been used before
        code = bufferedStream.Clear();
        if(code != ErrorCode.OK) {
            return "buffer failed to clear";
        }

        // Insert points into the buffer
        for (int i = 0; i < numberOfPoints; ++i) {
            double positionRot = points[i][0];
            double velocityRPM = points[i][1];
            int durationMilliseconds = (int) points[i][2];
            int direction = profile.forward? +1 : -1;

            // populate point values
            point.timeDur = durationMilliseconds;
            point.position = direction * positionRot * ArmConstants.kSensorUnitsPerRotation; // Convert Revolutions to Units
            point.velocity = direction * velocityRPM * ArmConstants.kSensorUnitsPerRotation / 600.0; // Convert RPM to Units/100ms
            point.auxiliaryPos = 0;
            point.auxiliaryVel = 0;
            point.profileSlotSelect0 = ArmConstants.kPrimaryPIDSlot; // set of gains you would like to use
            point.profileSlotSelect1 = 0; // auxiliary PID [0,1], leave zero
            point.zeroPos = (i == 0); // set this to true on the first point
            point.isLastPoint = ((i + 1) == numberOfPoints); // set this to true on the last point
            point.arbFeedFwd = 0; // you can add a constant offset to add to PID[0] output here - TODO implement

            code = bufferedStream.Write(point);
            if(code != ErrorCode.OK) {
                return "buffer failed to write point at index: " + i;
            }
        }

        return null;
    }

    private void logMotionProfileStatus(TalonFX talon) {
        // TODO implement logging, do so at reasonable intervals
        MotionProfileStatus status = new MotionProfileStatus();
        talon.getMotionProfileStatus(status);
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
        return ArmConstants.ThrottleAtFullExtension * (armExtension(proximalTicks, distalTicks) / ArmConstants.FullExtensionDistance);
    }
    private double getArbitraryFeedForwardForProximalArm(){
        return getArbitraryFeedForwardForProximalArm(proximalMotor.getSelectedSensorPosition(0), distalMotor.getSelectedSensorPosition(0));
    }
    private double getArbitraryFeedForwardForDistalArm(double distalTicks){
        return ArmConstants.ThrottleAtFullExtension * (distalArmExtension(distalTicks) / ArmConstants.distalArmLength);
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
        SmartDashboard.putNumber("Davids Distall Calc", 
        SmartDashboard.putNumber

    }

    public void resetArmEncoders() {
        proximalMotor.setSelectedSensorPosition((proximalAbsolute.getAverageValue()- ArmConstants.proximalAbsoluteTicsCenter) * ArmConstants.proximalRelativeTicsPerAbsoluteTick);
        distalMotor.setSelectedSensorPosition((distalAbsolute.getAverageValue()- ArmConstants.distalAbsoluteTicsCenter) * ArmConstants.distalRelativeTicsPerAbsoluteTick);

    }
    public void resetTrajectories () {

    }

    public void setArmMotors(double pAxis, double dAxis) {
        proximalMotor.set(ControlMode.PercentOutput, pAxis);
        distalMotor.set(ControlMode.PercentOutput, dAxis);
    }
}
