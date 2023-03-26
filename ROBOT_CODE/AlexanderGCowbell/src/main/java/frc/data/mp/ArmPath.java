package frc.data.mp;

import com.ctre.phoenix.motion.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.ArbitraryFeedForward;


public class ArmPath {
    private int numberOfPoints;
    private double[][] proximalPoints;
    private double[][] distalPoints;
    private int wristFlexIndex;
    private double wristFlexPosition;
    private double wristMaxVelocity;
    private int wristExtendIndex;
    private BufferedTrajectoryPointStream proximalBufferedStream;
    private BufferedTrajectoryPointStream distalBufferedStream;
    private int autoStartIndex;

    public enum ArmMotor {
        PROXIMAL, DISTAL, WRIST
    }

    public enum Direction {
        FORWARD, REVERSE
    }

    public ArmPath(int numberOfPoints, double[][] proximalPoints, double[][] distalPoints, int wristFlexIndex, double wristFlexPosition, int wristExtendIndex, double wristMaxVelocity) {
        this(numberOfPoints, proximalPoints, distalPoints, wristFlexIndex, wristFlexPosition, wristExtendIndex, wristMaxVelocity, 0);
    }
    
    public ArmPath(int numberOfPoints, double[][] proximalPoints, double[][] distalPoints, int wristFlexIndex, double wristFlexPosition, int wristExtendIndex, double wristMaxVelocity, int autoStartIndex) {
        this.numberOfPoints = numberOfPoints;
        this.proximalPoints = proximalPoints;
        this.distalPoints = distalPoints;
        this.wristFlexIndex = wristFlexIndex;
        this.wristFlexPosition = wristFlexPosition;
        this.wristExtendIndex = wristExtendIndex;
        this.wristMaxVelocity = wristMaxVelocity;
        this.proximalBufferedStream = new BufferedTrajectoryPointStream();
        this.distalBufferedStream = new BufferedTrajectoryPointStream();
        this.autoStartIndex = autoStartIndex;
    }
    public int getNumberOfPoints() {
        return numberOfPoints;
    }

    public int getWristFlexIndex() {
        return wristFlexIndex;
    }

    public double getWristFlexPosition() {
        return wristFlexPosition;
    }

    public int getWristExtendIndex() {
        return wristExtendIndex;
    }

    public double getWristMaxVelocity() {
        return wristMaxVelocity;
    }

    public int getAutoStartIndex() {
        return autoStartIndex;
    }


    public BufferedTrajectoryPointStream getInitializedBuffer(ArmMotor motor, int startFrom, Direction direction) {
        BufferedTrajectoryPointStream bufferedStream = (motor == ArmMotor.PROXIMAL)? proximalBufferedStream : distalBufferedStream;
        TrajectoryPoint point = new TrajectoryPoint(); 

        // clear the buffer, it may have been used before
        bufferedStream.Clear();

        // Insert points into the buffer
        if(direction == Direction.FORWARD) {
            for (int i = startFrom; i < numberOfPoints; ++i) {
                boolean isLastPoint = ((i + 1) == numberOfPoints);
                populateBufferPoint(motor, bufferedStream, point, i, isLastPoint, direction);
            }
        } else {
            for (int i = startFrom; i >= 0; --i) {
                boolean isLastPoint = (i == 0);
                populateBufferPoint(motor, bufferedStream, point, i, isLastPoint, direction);
            }
        }

        return bufferedStream;
    }

    private void populateBufferPoint(ArmMotor motor, BufferedTrajectoryPointStream bufferedStream, TrajectoryPoint point, int index, boolean isLastPoint, Direction direction) {
        double proximalPosition = proximalPoints[index][0];
        double distalPosition = distalPoints[index][0];
        double position = (motor == ArmMotor.PROXIMAL)? proximalPosition : distalPosition;
        double velocityRPM = (motor == ArmMotor.PROXIMAL)? proximalPoints[index][1] : distalPoints[index][1];
        int durationMilliseconds = ArmConstants.pointDurationMS;
        double arbFeedFwd = (motor == ArmMotor.PROXIMAL)? 
                ArbitraryFeedForward.getArbitraryFeedForwardForProximalArm(proximalPosition, distalPosition) : 
                ArbitraryFeedForward.getArbitraryFeedForwardForDistalArm(distalPosition);

        // populate point values
        point.timeDur = durationMilliseconds;
        point.position = position;
        point.velocity = direction == Direction.FORWARD? velocityRPM: -velocityRPM;
        point.auxiliaryPos = 0;
        point.auxiliaryVel = 0;
        point.profileSlotSelect0 = ArmConstants.kPrimaryPIDSlot; // set of gains you would like to use
        point.profileSlotSelect1 = 0; // auxiliary PID [0,1], leave zero
        point.isLastPoint = isLastPoint; // set this to true on the last point
        point.arbFeedFwd = arbFeedFwd; // you can add a constant offset to add to PID[0] output here

        bufferedStream.Write(point);
    }
}
