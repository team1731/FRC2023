package frc.data.mp;

import com.ctre.phoenix.motion.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.ArbitraryFeedForward;


public class ArmPath {
    private int numberOfPoints;
    private double [][]proximalPoints;
    private double [][]distalPoints;
    private double [][]wristPoints;
    private BufferedTrajectoryPointStream proximalBufferedStream;
    private BufferedTrajectoryPointStream distalBufferedStream;

    public enum ArmMotor {
        PROXIMAL, DISTAL, WRIST
    }

    public enum Direction {
        FORWARD, REVERSE
    }

    public ArmPath(int numberOfPoints, double [][]proximalPoints,double [][]distalPoints, double [][]wristPoints) {
        this.numberOfPoints = numberOfPoints;
        this.proximalPoints = proximalPoints;
        this.distalPoints = distalPoints;
        this.wristPoints = wristPoints;
        this.proximalBufferedStream = new BufferedTrajectoryPointStream();
        this.distalBufferedStream = new BufferedTrajectoryPointStream();
    }

    public int getNumberOfPoints() {
        return numberOfPoints;
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
                populateBufferPoint(motor, bufferedStream, point, i, isLastPoint);
            }
        } else {
            for (int i = startFrom; i >= 0; --i) {
                boolean isLastPoint = (i == 0);
                populateBufferPoint(motor, bufferedStream, point, i, isLastPoint);
            }
        }

        return bufferedStream;
    }

    public double[] getWristAtIndex(int index) {
        return wristPoints[index];
    }

    private void populateBufferPoint(ArmMotor motor, BufferedTrajectoryPointStream bufferedStream, TrajectoryPoint point, int index, boolean isLastPoint) {
        double proximalPosition = proximalPoints[index][0];
        double distalPosition = proximalPoints[index][0];
        double position = (motor == ArmMotor.PROXIMAL)? proximalPosition : distalPosition;
        double velocityRPM = (motor == ArmMotor.PROXIMAL)? proximalPoints[index][1] : distalPoints[index][1];
        int durationMilliseconds = (motor == ArmMotor.PROXIMAL)? (int)proximalPoints[index][2] : (int)distalPoints[index][2];
        double arbFeedFwd = (motor == ArmMotor.PROXIMAL)? 
                ArbitraryFeedForward.getArbitraryFeedForwardForProximalArm(proximalPosition, distalPosition) : 
                ArbitraryFeedForward.getArbitraryFeedForwardForDistalArm(distalPosition);

        // populate point values
        point.timeDur = durationMilliseconds;
        point.position = position;
        point.velocity = velocityRPM;
        point.auxiliaryPos = 0;
        point.auxiliaryVel = 0;
        point.profileSlotSelect0 = ArmConstants.kPrimaryPIDSlot; // set of gains you would like to use
        point.profileSlotSelect1 = 0; // auxiliary PID [0,1], leave zero
        point.isLastPoint = isLastPoint; // set this to true on the last point
        point.arbFeedFwd = arbFeedFwd; // you can add a constant offset to add to PID[0] output here

        bufferedStream.Write(point);
    }
}
