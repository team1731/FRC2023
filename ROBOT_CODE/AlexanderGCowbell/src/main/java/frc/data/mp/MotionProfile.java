package frc.data.mp;

public class MotionProfile {
    public int numberOfPoints;
    public double [][]points;

    public MotionProfile(int numberOfPoints, double [][]points) {
        this.numberOfPoints = numberOfPoints;
        this.points = points;
    }
}
