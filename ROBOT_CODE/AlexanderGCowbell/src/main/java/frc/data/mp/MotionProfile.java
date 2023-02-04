package frc.data.mp;

public class MotionProfile {
    public int numberOfPoints;
    public double [][]points;
    public boolean forward = true;

    public MotionProfile(int numberOfPoints, double [][]points, boolean forward) {
        this.numberOfPoints = numberOfPoints;
        this.points = points;
        this.forward = forward;
    }
}
