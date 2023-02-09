package frc.data.mp;

public class ArmPath {
    public int numberOfPoints;
    public double [][]proximalPoints;
    public double [][]distalPoints;
    public double [][]wristPoints;


    public ArmPath(int numberOfPoints, double [][]distalPoints, double [][]proximalPoints, double [][]wristPoints) {
        this.numberOfPoints = numberOfPoints;
        this.proximalPoints = proximalPoints;
        this.wristPoints = wristPoints;
    }
}
