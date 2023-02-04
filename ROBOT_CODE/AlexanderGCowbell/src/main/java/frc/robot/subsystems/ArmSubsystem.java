package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motion.*;
import com.ctre.phoenix.ErrorCode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
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
    private TalonFXConfiguration talonConfig;

    // motion profiles/buffers for arm proximal and distal motors
    private BufferedTrajectoryPointStream proximalBufferedStream;
    private BufferedTrajectoryPointStream distalBufferedStream;

    // motors for the wrist/hand
    private static final int deviceID = 1;
    private CANSparkMax wristMotor;
    private CANSparkMax handMotor;
    private SparkMaxPIDController wristPIDController;
    private RelativeEncoder wristEncoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

    // state tracking
    private boolean proximalMotorRunning = false;
    private boolean distalMotorRunning = false;



    public ArmSubsystem() {
        // setup motor and motion profiling members
        proximalMotor = new WPI_TalonFX(ArmConstants.proximalCancoderId, "canivore1");
        proximalMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 30, 0.2));
        distalMotor = new WPI_TalonFX(ArmConstants.distalCancoderId, "canivore1");
        distalMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 30, 0.2));
        talonConfig = new TalonFXConfiguration(); // factory default settings
        proximalBufferedStream = new BufferedTrajectoryPointStream();
        distalBufferedStream = new BufferedTrajectoryPointStream();

        // setup wrist/hand motor members
        wristMotor = new CANSparkMax(ArmConstants.wristCancoderId, MotorType.kBrushless);
        handMotor = new CANSparkMax(ArmConstants.handCancoderId, MotorType.kBrushless);

        // kick off initializations
        initializeArm();
        //initializeHand();
    }

    /*
     * METHODS FOR INITIALIZING AND MOVING THE ARM
     */

    // configures the arm motors, should be called 
    private void initializeArm() {
        // TODO clean up configuration code

        /* _config the master specific settings */
        talonConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        talonConfig.neutralDeadband = ArmConstants.kNeutralDeadband; /* 0.1 % super small for best low-speed control */
        talonConfig.slot0.kF = ArmConstants.kGains_MotProf.kF;
        talonConfig.slot0.kP = ArmConstants.kGains_MotProf.kP;
        talonConfig.slot0.kI = ArmConstants.kGains_MotProf.kI;
        talonConfig.slot0.kD = ArmConstants.kGains_MotProf.kD;
        talonConfig.slot0.integralZone = (int) ArmConstants.kGains_MotProf.kIzone;
        talonConfig.slot0.closedLoopPeakOutput = ArmConstants.kGains_MotProf.kPeakOutput;
        // _config.slot0.allowableClosedloopError // left default for this example
        // _config.slot0.maxIntegralAccumulator; // left default for this example
        // _config.slot0.closedLoopPeriod; // left default for this example
        proximalMotor.configAllSettings(talonConfig);
        distalMotor.configAllSettings(talonConfig);

        /* pick the sensor phase and desired direction */
        proximalMotor.setInverted(TalonFXInvertType.CounterClockwise);
        distalMotor.setInverted(TalonFXInvertType.CounterClockwise);

        /*
        proximalMotor.configSelectedFeedbackSensor(
			TalonFXFeedbackDevice.IntegratedSensor, // Sensor Type
			OpConstants.kPIDLoopIdx, // PID Index
			OpConstants.kTimeoutMs); // Config Timeout

		distalMotor.configSelectedFeedbackSensor(
			TalonFXFeedbackDevice.IntegratedSensor, // Sensor Type
			OpConstants.kPIDLoopIdx, // PID Index
			OpConstants.kTimeoutMs); // Config Timeout
        
        proximalMotor.configNeutralDeadband(0.001, OpConstants.kTimeoutMs);
		_swingerSlaveMotor.configNeutralDeadband(0.001, OpConstants.kTimeoutMs);


		proximalMotor.setSensorPhase(true);
		proximalMotor.setInverted(false);
		distalMotor.setSensorPhase(true);
		distalMotor.setInverted(false);

		proximalMotor.configNominalOutputForward(0, OpConstants.kTimeoutMs);
		proximalMotor.configNominalOutputReverse(0, OpConstants.kTimeoutMs);
		proximalMotor.configPeakOutputForward(1, OpConstants.kTimeoutMs);
		proximalMotor.configPeakOutputReverse(-1, OpConstants.kTimeoutMs);

		distalMotor.configNominalOutputForward(0, OpConstants.kTimeoutMs);
		distalMotor.configNominalOutputReverse(0, OpConstants.kTimeoutMs);
		distalMotor.configPeakOutputForward(1, OpConstants.kTimeoutMs);
		distalMotor.configPeakOutputReverse(-1, OpConstants.kTimeoutMs);

		proximalMotor.selectProfileSlot(OpConstants.SLOT_0, OpConstants.kPIDLoopIdx);
		proximalMotor.config_kP(OpConstants.SLOT_0, ClimbConstants.kP, OpConstants.kTimeoutMs);
		proximalMotor.config_kI(OpConstants.SLOT_0, ClimbConstants.kI, OpConstants.kTimeoutMs);
		proximalMotor.config_kD(OpConstants.SLOT_0, ClimbConstants.kD, OpConstants.kTimeoutMs);
		proximalMotor.config_kF(OpConstants.SLOT_0, ClimbConstants.kFF, OpConstants.kTimeoutMs);

		distalMotor.selectProfileSlot(OpConstants.SLOT_0, OpConstants.kPIDLoopIdx);
		distalMotor.config_kP(OpConstants.SLOT_0, ClimbConstants.kP, OpConstants.kTimeoutMs);
		distalMotor.config_kI(OpConstants.SLOT_0, ClimbConstants.kI, OpConstants.kTimeoutMs);
		distalMotor.config_kD(OpConstants.SLOT_0, ClimbConstants.kD, OpConstants.kTimeoutMs);
		distalMotor.config_kF(OpConstants.SLOT_0, ClimbConstants.kFF, OpConstants.kTimeoutMs);

        proximalMotor.setSelectedSensorPosition(0, OpConstants.kPIDLoopIdx, 0);
		distalMotor.setSelectedSensorPosition(0, OpConstants.kPIDLoopIdx, 0);
        */
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
        proximalMotor.set(0.5);
        code = proximalMotor.startMotionProfile(proximalBufferedStream, 10, TalonFXControlMode.MotionProfile.toControlMode());
        if(code != ErrorCode.OK) {
            notifyStateMachine(ResultCode.ARM_MOTION_START_FAILED, "Failed to start proximal motion profile");
            return;
        }

        distalMotorRunning = true;
        distalMotor.set(0.5);
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

    /*
     * METHODS FOR INITIALIZING AND MOVING THE HAND/WRIST
     */
    /*
    public void initializeHand() {
        // initialize motor
        wristMotor = new CANSparkMax(deviceID, MotorType.kBrushless);
        wristMotor.restoreFactoryDefaults();

        // initialze PID controller and encoder objects
        wristPIDController = wristMotor.getPIDController();
        wristEncoder = wristMotor.getEncoder();

        // PID coefficients
        kP = 5e-5; 
        kI = 1e-6;
        kD = 0; 
        kIz = 0; 
        kFF = 0.000156; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;

        // Smart Motion Coefficients
        maxVel = 2000; // rpm
        maxAcc = 1500;

        // set PID coefficients
        wristPIDController.setP(kP);
        wristPIDController.setI(kI);
        wristPIDController.setD(kD);
        wristPIDController.setIZone(kIz);
        wristPIDController.setFF(kFF);
        wristPIDController.setOutputRange(kMinOutput, kMaxOutput);

        //
         // Smart Motion coefficients are set on a SparkMaxPIDController object
         // 
         // - setSmartMotionMaxVelocity() will limit the velocity in RPM of
         // the pid controller in Smart Motion mode
         // - setSmartMotionMinOutputVelocity() will put a lower bound in
         // RPM of the pid controller in Smart Motion mode
         // - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
         // of the pid controller in Smart Motion mode
         // - setSmartMotionAllowedClosedLoopError() will set the max allowed
         // error for the pid controller in Smart Motion mode
        //
        int smartMotionSlot = 0;
        wristPIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        wristPIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        wristPIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        wristPIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

        //wristPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
        
    }
    */


    /*
     * PERIODIC LOGIC
     */

    @Override
    public void periodic() {
        boolean isArmMovingAtPeriodicStart = isArmMoving();

        if(stateMachine != null) {
            stateMachine.periodic(); // prompt the state machine to process any periodic tasks
        }

        if(proximalMotorRunning && proximalMotor.isMotionProfileFinished()) {
            proximalMotor.set(0);
            proximalMotorRunning = false;
        } else if(proximalMotorRunning) {
            logMotionProfileStatus(proximalMotor);
        }

        if(distalMotorRunning && distalMotor.isMotionProfileFinished()) {
            distalMotor.set(0);
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
        boolean forward = true; // TODO, determine if we will use this, or use negative values in motion profile
        TrajectoryPoint point = new TrajectoryPoint(); 
        ErrorCode code;

        // clear the buffer, it may have been used before
        code = bufferedStream.Clear();
        if(code != ErrorCode.OK) {
            return "buffer failed to clear";
        }

        // Insert points into the buffer
        for (int i = 0; i < numberOfPoints; ++i) {

            double direction = forward ? +1 : -1;
            double positionRot = points[i][0];
            double velocityRPM = points[i][1];
            int durationMilliseconds = (int) points[i][2];

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
}
