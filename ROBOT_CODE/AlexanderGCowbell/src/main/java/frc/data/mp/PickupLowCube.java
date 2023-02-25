package frc.data.mp;

public class PickupLowCube {
    public static ArmPath getArmPath() {
        return new ArmPath(kNumPoints, proximalPoints, distalPoints, kWristFlexIndex, kWristFlexPosition, kWristExtendIndex, kWristMaxVelocity);
    }

    public static final int kNumPoints = 2;
    public static final int kWristFlexIndex = 1;
    public static final double kWristFlexPosition = 0.35;
    public static final int kWristExtendIndex = 1;
    public static final double kWristMaxVelocity = 3000;

    // Position (ticks)	Velocity (RPM)

    public static double [][]proximalPoints = new double[][]{
        {-4388.0,  1.0},
        {-4389.0,  0.0},
        
   
    };

    public static double [][]distalPoints = new double[][]{
        { 10300.0,  1.0},
        { 10301.0,  0.0},

    };

    public static double []wristPoints = new double[]{
         0.35,
         0.35,
  
    };

};