package frc.data.mp;

public class PickupLowCone {
    public static ArmPath getArmPath() {
        return new ArmPath(kNumPoints, proximalPoints, distalPoints, kWristFlexIndex, kWristFlexPosition, kWristExtendIndex, kWristMaxVelocity);
    }

    public static final int kNumPoints = 12;
    public static final int kWristFlexIndex = 5;
    public static final double kWristFlexPosition = 0.35;
    public static final int kWristExtendIndex = 5;
    public static final double kWristMaxVelocity = 3000;

    // Position (ticks)	Velocity (RPM)

    public static double [][]proximalPoints = new double[][]{
        {-4388.0,  0.0},
        {-4388.0,  0.0},
        {-4388.0,  0.0},
        {-4388.0,  0.0},
        {-4388.0,  0.0},
        {-4388.0,  0.0},
        {-4388.0,  0.0},
        {-4388.0,  0.0},
        {-4388.0,  0.0},
        {-4388.0,  0.0},
        {-4388.0,  0.0},
        {-4388.0,  0.0},
   
    };

    public static double [][]distalPoints = new double[][]{
        { 10300.0,  0.0},
        { 10300.0,  0.0},
        { 10300.0,  0.0},
        { 10300.0,  0.0},
        { 10300.0,  0.0},
        { 10300.0,  0.0},
        { 10300.0,  0.0},
        { 10300.0,  0.0},
        { 10300.0,  0.0},
        { 10300.0,  0.0},
        { 10300.0,  0.0},
        { 10300.0,  0.0},




    };

    public static double []wristPoints = new double[]{
         0.35,
         0.35,
  
    };

};
