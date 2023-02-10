package frc.data.mp;
	
public class ShelfIntakePath {	
    public static ArmPath getArmPath() {
        return new ArmPath(kNumPoints, proximalPoints, distalPoints, wristPoints);
    }
    
    public static final int kNumPoints = 2;		

    // Position (ticks)	Velocity (RPM)
    public static double [][]proximalPoints = new double[][]{		
        {0,	0},
        {0.000392857142857143,	4.714285714}           
    };
    
    public static double [][]distalPoints = new double[][]{		
        {0,	0},
        {0.000392857142857143,	4.714285714}           
    };

    public static double []wristPoints = new double[]{		
        0,
        0.000392857142857143          
    };
}			
