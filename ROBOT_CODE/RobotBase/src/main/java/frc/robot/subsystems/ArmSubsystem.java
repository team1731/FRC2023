package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motion.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state.arm.ArmStateMachine;
import frc.robot.state.*;
import frc.robot.Constants.ArmConstants;
import frc.data.mp.*;

public class ArmSubsystem extends SubsystemBase {
    // motors for the arm
    private WPI_TalonFX proximalMotor;
    private WPI_TalonFX distalMotor;
    private TalonFXConfiguration talonConfig;

    // motor for the wrist/hand
    // TODO implement

    // buffer for the motion profile values
    private BufferedTrajectoryPointStream mpBufferedStream;
    // the current raw motion profile values
    private double[][] mpPoints;
    private int mpNumPoints;

    // state tracking
    private boolean isArmMoving = false;
    private boolean printMotionValues = false;



    public ArmSubsystem() {
        proximalMotor = new WPI_TalonFX(ArmConstants.proximalCancoderId, "canivore1");
        proximalMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 30, 0.2));
        distalMotor = new WPI_TalonFX(ArmConstants.distalCancoderId, "canivore1");
        distalMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 30, 0.2));
        talonConfig = new TalonFXConfiguration(); // factory default settings
        mpBufferedStream = new BufferedTrajectoryPointStream();
    }

    public void initialize() {
        initializeArm();
    }

    /*
     * METHODS FOR INITIALIZING AND MOVING THE ARM
     */

    private void initializeArm() {
        /* fill our buffer object with the motion profile points */
        mpPoints = TestProfile.Points;
        mpNumPoints = TestProfile.kNumPoints;
        initBuffer(mpPoints, mpNumPoints);

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

        /* pick the sensor phase and desired direction */
        proximalMotor.setInverted(TalonFXInvertType.CounterClockwise);
    }

    public void initializeArmMovement() {
        proximalMotor.set(0.5);
        Instrum.printLine("Proximal motor initialized");
    }

    public void moveArm() {
        isArmMoving = true;
        proximalMotor.startMotionProfile(mpBufferedStream, 10, TalonFXControlMode.MotionProfile.toControlMode());
        Instrum.printLine("Proximal motor motion started");
    }

    public boolean isArmMoving() {
        return isArmMoving;
    }

    @Override
    public void periodic() {
        if(isArmMoving && proximalMotor.isMotionProfileFinished()) {
            isArmMoving = false;
            proximalMotor.set(0);
            Instrum.printLine("Proximal motor motion finished");
        }

        Instrum.loop(printMotionValues, proximalMotor);
    }

    public void printMotionValues(boolean printValues) {
        printMotionValues = printValues;
    }


    /**
     * BUFFER HANDLING
     *
     * @param profile  generated array of points
     * @param totalCnt num points in profile
     */
    private void initBuffer(double[][] profile, int totalCnt) {

        boolean forward = true; // set to false to drive in opposite direction of profile (not really needed
                                // since you can use negative numbers in profile).

        TrajectoryPoint point = new TrajectoryPoint(); // temp for for loop, since unused params are initialized
                                                       // automatically, you can alloc just one

        /* clear the buffer, in case it was used elsewhere */
        mpBufferedStream.Clear();

        /* Insert every point into buffer, no limit on size */
        for (int i = 0; i < totalCnt; ++i) {

            double direction = forward ? +1 : -1;
            double positionRot = profile[i][0];
            double velocityRPM = profile[i][1];
            int durationMilliseconds = (int) profile[i][2];

            /* for each point, fill our structure and pass it to API */
            point.timeDur = durationMilliseconds;
            point.position = direction * positionRot * ArmConstants.kSensorUnitsPerRotation; // Convert Revolutions to
                                                                                          // Units
            point.velocity = direction * velocityRPM * ArmConstants.kSensorUnitsPerRotation / 600.0; // Convert RPM to
                                                                                                  // Units/100ms
            point.auxiliaryPos = 0;
            point.auxiliaryVel = 0;
            point.profileSlotSelect0 = ArmConstants.kPrimaryPIDSlot; /* which set of gains would you like to use [0,3]? */
            point.profileSlotSelect1 = 0; /* auxiliary PID [0,1], leave zero */
            point.zeroPos = (i == 0); /* set this to true on the first point */
            point.isLastPoint = ((i + 1) == totalCnt); /* set this to true on the last point */
            point.arbFeedFwd = 0; /* you can add a constant offset to add to PID[0] output here */

            mpBufferedStream.Write(point);
        }
    }

    static class Instrum {
        static int _loops = 0;
        static boolean _bPrintValues = false;
        
        public static void printLine(String s) {
            System.out.println(s);
        }
    
        public static void loop(boolean bPrintValues, TalonFX talon) {
            if (!_bPrintValues && bPrintValues) {
                /* user just pressed button, immediete print */
                _loops = 999;
            }
            /* if button is off, don't print */
            if (bPrintValues == false) {
                /* reset so we don't print */
                _loops = 0;
            }
            /* save for next compare */
            _bPrintValues = bPrintValues;
    
            /* build string and print if button is down */
            if (++_loops >= 10) {
                _loops = 0;
                /* get status info */
                MotionProfileStatus status = new MotionProfileStatus();
                talon.getMotionProfileStatus(status);
    
                String line = "";
                line += "  topBufferRem: " + status.topBufferRem + "\n";
                line += "  topBufferCnt: " + status.topBufferCnt + "\n";
                line += "  btmBufferCnt: " + status.btmBufferCnt + "\n";
                line += "  hasUnderrun: " + status.hasUnderrun + "\n";
                line += "  isUnderrun: " + status.isUnderrun + "\n";
                line += "  activePointValid: " + status.activePointValid + "\n";
                line += "  isLast: " + status.isLast + "\n";
                line += "  profileSlotSelect0: " + status.profileSlotSelect + "\n";
                line += "  profileSlotSelect1: " + status.profileSlotSelect1 + "\n";
                line += "  outputEnable: " + status.outputEnable.toString() + "\n";
                line += "  timeDurMs: " + status.timeDurMs + "\n";
    
                printLine(line);
            }
        }
    }    
}
