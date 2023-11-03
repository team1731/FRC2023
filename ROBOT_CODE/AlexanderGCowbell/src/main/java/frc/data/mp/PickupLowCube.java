package frc.data.mp;

public class PickupLowCube {
    public static ArmPath getArmPath() {
        return new ArmPath(kNumPoints, proximalPoints, distalPoints, kWristFlexIndex, kWristFlexPosition, kWristExtendIndex, kWristMaxVelocity);
    }

    public static final int kNumPoints = 57;
    public static final int kWristFlexIndex = 10;
    public static final double kWristFlexPosition = 0.51;
    public static final int kWristExtendIndex = 56;
    public static final double kWristMaxVelocity = 2000;

    // Position (ticks)	Velocity (RPM)

   
  
    public static double [][]proximalPoints = new double[][]{
        {-4388, -27},
        {-4412.672619, -139},
        {-4348.278571, -161.4},
        {-4305.017857, -205},
        {-4228.357143, -215},
        {-4159.696429, -232.6},
        {-4091.035714, -250.2},
        {-4022.375, -267.8},
        {-3945.314286, -272.4},
        {-3868.253571, -277},
        {-3783.192857, -255.6},
        {-3694.732143, -234.2},
        {-3606.871429, -194.6},
        {-3517.410714, -158},
        {-3427.95, -111.4},
        {-3338.489286, -83.2},
        {-3252.428571, -55},
        {-3165.767857, -37},
        {-3079.107143, -19},
        {-2992.446429, -9},
        {-2905.785714, 1},
        {-2819.925, 6.6},
        {-2734.064286, 4.2},
        {-2652.003571, -1.2},
        {-2569.942857, -8.6},
        {-2492.682143, -20.8},
        {-2414.621429, -28.6},
        {-2336.560714, -36.8},
        {-2254.7, -42},
        {-2172.639286, -46},
        {-2085.778571, -45.2},
        {-1998.717857, -40.6},
        {-1911.657143, -35.6},
        {-1824.596429, -30.6},
        {-1737.735714, -22},
        {-1650.875, -13.4},
        {-1564.214286, -6.2},
        {-1477.553571, 1},
        {-1390.892857, 6.2},
        {-1304.232143, 6.6},
        {-1217.571429, 6.6},
        {-1130.910714, 4.2},
        {-1044.25, 1.6},
        {-957.5892857, 1},
        {-870.9285714, 0.4},
        {-784.2678571, 0.2},
        {-697.6071429, 0},
        {-610.9464286, 0},
        {-524.2857143, 0},
        {-437.625, 0},
        {-350.9642857, 0},
        {-264.3035714, 0},
        {-177.6428571, 0},
        {-90.98214286, 0},
        {-4.321428571, 0},
        {82.33928571, 0},
        {169, 0}
    };

    public static double [][]distalPoints = new double[][]{
        {10300, 10},
        {10441.77976, 54.66666667},
        {10545.69286, 63.6},
        {10698.53929, 120.2},
        {10837.18571, 163.4},
        {11048.23214, 278.4},
        {11259.27857, 393.4},
        {11470.325, 508.4},
        {11722.77143, 665.4},
        {11975.21786, 822.4},
        {12245.66429, 995.8},
        {12516.11071, 1169.2},
        {12882.95714, 1425.4},
        {13164.20357, 1596.4},
        {13550.85, 1829.2},
        {13847.09643, 1973.8},
        {14143.34286, 2118.4},
        {14458.58929, 2223.4},
        {14773.83571, 2328.4},
        {15099.68214, 2402.6},
        {15425.52857, 2476.8},
        {15862.375, 2572.4},
        {16183.82143, 2624.8},
        {16610.06786, 2685.6},
        {16920.31429, 2715.4},
        {17327.96071, 2738.2},
        {17624.60714, 2739.6},
        {18012.25357, 2717.2},
        {18295.1, 2686.4},
        {18748.74643, 2598.6},
        {19104.99286, 2517.8},
        {19546.83929, 2417.8},
        {19897.68571, 2341.6},
        {20248.53214, 2265.4},
        {20510.77857, 2230.6},
        {20773.025, 2195.8},
        {21024.67143, 2164.4},
        {21276.31786, 2133},
        {21595.56429, 2083.2},
        {21832.61071, 2049},
        {22128.45714, 1989.4},
        {22349.30357, 1945.6},
        {22618.95, 1865.4},
        {22820.99643, 1803.6},
        {23063.04286, 1699.8},
        {23246.28929, 1621.4},
        {23461.33571, 1499.6},
        {23627.58214, 1414.2},
        {23818.22857, 1285.4},
        {23968.875, 1198.6},
        {24136.72143, 1070.2},
        {24272.76786, 985.2},
        {24416.41429, 859.2},
        {24535.66071, 776.6},
        {24653.90714, 652.8},
        {24762.8869, 543.3333333},
        {24854, 406}
    };

    public static double []wristPoints = new double[]{
        0.490721941,
        0.490721941,
        0.490721941,
        0.490721941,
        0.490721941,
        0.490721941,
        0.490721941,
        0.490721941,
        0.490721941,
        0.490721941,
        0.441850424,
        0.441850424,
        0.441850424,
        0.441850424,
        0.441850424,
        0.441850424,
        0.441850424,
        0.441850424,
        0.441850424,
        0.441850424,
        0.441850424,
        0.441850424,
        0.441850424,
        0.441850424,
        0.441850424,
        0.441850424,
        0.441850424,
        0.441850424,
        0.441850424,
        0.441850424,
        0.35020113,
        0.35020113,
        0.35020113,
        0.35020113,
        0.35020113,
        0.35020113,
        0.35020113,
        0.35020113,
        0.35020113,
        0.35020113,
        0.35020113,
        0.35020113,
        0.35020113,
        0.35020113,
        0.35020113,
        0.35020113,
        0.35020113,
        0.35020113,
        0.35020113,
        0.35020113,
        0.28768152,
        0.28768152,
        0.28768152,
        0.28768152,
        0.28768152,
        0.28768152,
        0.28768152
    };

};
