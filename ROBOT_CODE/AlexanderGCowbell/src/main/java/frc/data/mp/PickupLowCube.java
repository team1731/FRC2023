package frc.data.mp;

public class PickupLowCube {
    public static ArmPath getArmPath() {
        return new ArmPath(kNumPoints, proximalPoints, distalPoints, kWristFlexIndex, kWristFlexPosition, kWristExtendIndex, kWristMaxVelocity);
    }

    public static final int kNumPoints = 57;
    public static final int kWristFlexIndex = 5;
    public static final double kWristFlexPosition = 0.58;
    public static final int kWristExtendIndex = 56;
    public static final double kWristMaxVelocity = 2000;

    // Position (ticks)	Velocity (RPM)

   
  
    public static double [][]proximalPoints = new double[][]{
        {-4388, -27},
        {-4418.708333, -139},
        {-4360.35, -161.4},
        {-4323.125, -205},
        {-4252.5, -215},
        {-4189.875, -232.6},
        {-4127.25, -250.2},
        {-4064.625, -267.8},
        {-3993.6, -272.4},
        {-3922.575, -277},
        {-3843.55, -255.6},
        {-3761.125, -234.2},
        {-3679.3, -194.6},
        {-3595.875, -158},
        {-3512.45, -111.4},
        {-3429.025, -83.2},
        {-3349, -55},
        {-3268.375, -37},
        {-3187.75, -19},
        {-3107.125, -9},
        {-3026.5, 1},
        {-2946.675, 6.6},
        {-2866.85, 4.2},
        {-2790.825, -1.2},
        {-2714.8, -8.6},
        {-2643.575, -20.8},
        {-2571.55, -28.6},
        {-2499.525, -36.8},
        {-2423.7, -42},
        {-2347.675, -46},
        {-2266.85, -45.2},
        {-2185.825, -40.6},
        {-2104.8, -35.6},
        {-2023.775, -30.6},
        {-1942.95, -22},
        {-1862.125, -13.4},
        {-1781.5, -6.2},
        {-1700.875, 1},
        {-1620.25, 6.2},
        {-1539.625, 6.6},
        {-1459, 6.6},
        {-1378.375, 4.2},
        {-1297.75, 1.6},
        {-1217.125, 1},
        {-1136.5, 0.4},
        {-1055.875, 0.2},
        {-975.25, 0},
        {-894.625, 0},
        {-814, 0},
        {-733.375, 0},
        {-652.75, 0},
        {-572.125, 0},
        {-491.5, 0},
        {-410.875, 0},
        {-330.25, 0},
        {-249.625, 0},
        {-169, 0}
    };

    public static double [][]distalPoints = new double[][]{
        {10300, 10},
        {10355.29762, 54.66666667},
        {10372.72857, 63.6},
        {10439.09286, 120.2},
        {10491.25714, 163.4},
        {10615.82143, 278.4},
        {10740.38571, 393.4},
        {10864.95, 508.4},
        {11030.91429, 665.4},
        {11196.87857, 822.4},
        {11380.84286, 995.8},
        {11564.80714, 1169.2},
        {11845.17143, 1425.4},
        {12039.93571, 1596.4},
        {12340.1, 1829.2},
        {12549.86429, 1973.8},
        {12759.62857, 2118.4},
        {12988.39286, 2223.4},
        {13217.15714, 2328.4},
        {13456.52143, 2402.6},
        {13695.88571, 2476.8},
        {14046.25, 2572.4},
        {14281.21429, 2624.8},
        {14620.97857, 2685.6},
        {14844.74286, 2715.4},
        {15165.90714, 2738.2},
        {15376.07143, 2739.6},
        {15677.23571, 2717.2},
        {15873.6, 2686.4},
        {16240.76429, 2598.6},
        {16510.52857, 2517.8},
        {16865.89286, 2417.8},
        {17130.25714, 2341.6},
        {17394.62143, 2265.4},
        {17570.38571, 2230.6},
        {17746.15, 2195.8},
        {17911.31429, 2164.4},
        {18076.47857, 2133},
        {18309.24286, 2083.2},
        {18459.80714, 2049},
        {18669.17143, 1989.4},
        {18803.53571, 1945.6},
        {18986.7, 1865.4},
        {19102.26429, 1803.6},
        {19257.82857, 1699.8},
        {19354.59286, 1621.4},
        {19483.15714, 1499.6},
        {19562.92143, 1414.2},
        {19667.08571, 1285.4},
        {19731.25, 1198.6},
        {19812.61429, 1070.2},
        {19862.17857, 985.2},
        {19919.34286, 859.2},
        {19952.10714, 776.6},
        {19983.87143, 652.8},
        {20006.36905, 543.3333333},
        {20011, 406}
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
