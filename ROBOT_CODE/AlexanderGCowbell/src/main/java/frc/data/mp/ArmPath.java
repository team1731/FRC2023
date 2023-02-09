package frc.data.mp;

import com.ctre.phoenix.motion.*;
import frc.robot.Constants.ArmConstants;


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
        double[][] points = motor == ArmMotor.PROXIMAL? proximalPoints : distalPoints;
        BufferedTrajectoryPointStream bufferedStream = motor == ArmMotor.PROXIMAL? proximalBufferedStream : distalBufferedStream;
        TrajectoryPoint point = new TrajectoryPoint(); 

        // clear the buffer, it may have been used before
        bufferedStream.Clear();

        // Insert points into the buffer
        if(direction == Direction.FORWARD) {
            for (int i = startFrom; i < numberOfPoints; ++i) {
                boolean isLastPoint = ((i + 1) == numberOfPoints);
                populateBufferPoint(bufferedStream, point, points, i, isLastPoint);
            }
        } else {
            for (int i = startFrom; i >= 0; --i) {
                boolean isLastPoint = i == 0;
                populateBufferPoint(bufferedStream, point, points, i, isLastPoint);
            }
        }

        return bufferedStream;
    }

    public double[] getWristAtIndex(int index) {
        return wristPoints[index];
    }

    private void populateBufferPoint(BufferedTrajectoryPointStream bufferedStream, TrajectoryPoint point, double[][] points, int index, boolean isLastPoint) {
        double position = points[index][0];
        double velocityRPM = points[index][1];
        int durationMilliseconds = (int) points[index][2];

        // populate point values
        point.timeDur = durationMilliseconds;
        point.position = position;
        point.velocity = velocityRPM;
        point.auxiliaryPos = 0;
        point.auxiliaryVel = 0;
        point.profileSlotSelect0 = ArmConstants.kPrimaryPIDSlot; // set of gains you would like to use
        point.profileSlotSelect1 = 0; // auxiliary PID [0,1], leave zero
        point.isLastPoint = isLastPoint; // set this to true on the last point
        point.arbFeedFwd = 0; // you can add a constant offset to add to PID[0] output here

        bufferedStream.Write(point);
    }
}
