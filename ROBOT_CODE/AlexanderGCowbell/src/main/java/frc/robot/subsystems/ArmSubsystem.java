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
import com.revrobotics.CANSparkMax.IdleMode;

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

    // motion profiles/buffers for arm proximal and distal motors
    private BufferedTrajectoryPointStream proximalBufferedStream;
    private BufferedTrajectoryPointStream distalBufferedStream;

    // motors for the wrist/hand
    private CANSparkMax wristMotor;
    private CANSparkMax intakeMotor;
    private SparkMaxPIDController wristPIDController;
    private RelativeEncoder wristEncoder;

    // state tracking
    private boolean proximalMotorRunning = false;
    private boolean distalMotorRunning = false;



    public ArmSubsystem() {
        initializeArmMotors();
        //initializeWristMotor();
        //initializeIntakeMotor();
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

    /*
     * METHODS FOR INITIALIZING AND MOVING THE ARM
     */

    private void initializeArmMotors() {
        proximalMotor = new WPI_TalonFX(ArmConstants.proximalCancoderId, "canivore1");
        initializeTalonMotor(proximalMotor, TalonFXInvertType.CounterClockwise);
        proximalBufferedStream = new BufferedTrajectoryPointStream();

        distalMotor = new WPI_TalonFX(ArmConstants.distalCancoderId, "canivore1");
        initializeTalonMotor(distalMotor, TalonFXInvertType.CounterClockwise);
        distalBufferedStream = new BufferedTrajectoryPointStream();
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
        // talonConfig.slot0.allowableClosedloopError // left default for this example
        // talonConfig.slot0.maxIntegralAccumulator; // left default for this example
        // talonConfig.slot0.closedLoopPeriod; // left default for this example
        motor.configAllSettings(talonConfig);
        motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 30, 0.2));
        motor.setInverted(invertType);
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

    public boolean isArmMoving() {
        return proximalMotorRunning || distalMotorRunning;
    }

    public void resetSensorPosition() {
        proximalMotor.setSelectedSensorPosition(0);
        distalMotor.setSelectedSensorPosition(0);
    }


    /*
     * METHODS FOR INITIALIZING THE WRIST/INTAKE
     */

    public void initializeWristMotor() {
        wristMotor = new CANSparkMax(ArmConstants.wristCancoderId, MotorType.kBrushless);
        wristPIDController = wristMotor.getPIDController();
        wristEncoder = wristMotor.getEncoder();

        // initialize motor
        wristMotor.restoreFactoryDefaults();

        // set PID coefficients
        wristPIDController.setP(ArmConstants.handP);
        wristPIDController.setI(ArmConstants.handI);
        wristPIDController.setD(ArmConstants.handD);
        wristPIDController.setIZone(ArmConstants.handIz);
        wristPIDController.setFF(ArmConstants.handFF);
        wristPIDController.setOutputRange(ArmConstants.handMinOutput, ArmConstants.handMaxOutput);

        // set smart motion coefficients
        int smartMotionSlot = 0;
        wristPIDController.setSmartMotionMaxVelocity(ArmConstants.handMaxVel, smartMotionSlot);
        wristPIDController.setSmartMotionMinOutputVelocity(ArmConstants.handMinVel, smartMotionSlot);
        wristPIDController.setSmartMotionMaxAccel(ArmConstants.handMaxAcc, smartMotionSlot);
        wristPIDController.setSmartMotionAllowedClosedLoopError(ArmConstants.handAllowedErr, smartMotionSlot);
        //wristMotor.setSmartCurrentLimit(value??); TODO implement, what should this be?
    }

    public void initializeIntakeMotor() {
        intakeMotor = new CANSparkMax(ArmConstants.intakeCancoderId, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setInverted(false);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        //intakeMotor.setSmartCurrentLimit(value??); TODO implement, what should this be?
    }

    public void moveWrist(double rotations) {
        // TODO handle errors
        wristPIDController.setReference(rotations, CANSparkMax.ControlType.kSmartMotion);
        System.out.println("Wrist position " + wristEncoder.getPosition());
    }


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

        if(isArmMoving()) { // arm is still moving
            // TODO implement, check for issues? timeout?
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
}
