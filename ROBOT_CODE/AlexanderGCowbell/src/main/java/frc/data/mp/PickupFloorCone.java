package frc.data.mp;

public class PickupFloorCone {
    public static ArmPath getArmPath() {
        return new ArmPath(kNumPoints, proximalPoints, distalPoints, kWristFlexIndex, kWristFlexPosition, kWristExtendIndex, kWristMaxVelocity);
    }

    public static final int kNumPoints = 94;
    public static final int kWristFlexIndex = 1;
    public static final double kWristFlexPosition = 0.455;
    public static final int kWristExtendIndex = 93;
    public static final double kWristMaxVelocity = 2000;

    // Position (ticks)	Velocity (RPM)

   
    public static double [][]proximalPoints = new double[][]{
        {-7839, -4},
        {-7839, -4},
        {-7948, -107},
        {-7948, -107},
        {-8186, -341},
        {-8186, -341},
        {-8434, -591},
        {-8434, -591},
        {-8639, -798},
        {-8639, -798},
        {-8791, -947},
        {-8791, -947},
        {-8887, -943},
        {-8887, -943},
        {-8948, -772},
        {-8948, -772},
        {-8952, -527},
        {-8952, -527},
        {-8924, -293},
        {-8924, -293},
        {-8904, -119},
        {-8904, -119},
        {-8920, -35},
        {-8920, -35},
        {-8920, 27},
        {-8920, 27},
        {-8899, 50},
        {-8899, 50},
        {-8674, 236},
        {-8674, 236},
        {-8173, 707},
        {-8173, 707},
        {-7646, 1251},
        {-7646, 1251},
        {-7161, 1739},
        {-7161, 1739},
        {-6734, 2150},
        {-6734, 2150},
        {-6334, 2338},
        {-6334, 2338},
        {-5866, 2307},
        {-5866, 2307},
        {-5300, 2343},
        {-5300, 2343},
        {-4749, 2409},
        {-4749, 2409},
        {-4247, 2484},
        {-4247, 2484},
        {-3808, 2525},
        {-3808, 2525},
        {-3424, 2451},
        {-3424, 2451},
        {-3063, 2245},
        {-3063, 2245},
        {-2594, 2153},
        {-2594, 2153},
        {-1944, 2295},
        {-1944, 2295},
        {-1250, 2546},
        {-1250, 2546},
        {-603, 2809},
        {-603, 2809},
        {2, 3057},
        {2, 3057},
        {545, 3142},
        {545, 3142},
        {1029, 2983},
        {1029, 2983},
        {1445, 2708},
        {1445, 2708},
        {1445, 2708},
        {1626, 1658},
        {1626, 1658},
        {1394, 880},
        {1394, 880},
        {1206, 203},
        {1206, 203},
        {1065, -358},
        {1065, -358},
        {964, -759},
        {964, -759},
        {910, -723},
        {910, -723},
        {868, -533},
        {868, -533},
        {868, -533},
        {871, -343},
        {871, -343},
        {878, -191},
        {878, -191},
        {873, -94},
        {873, -94},
        {875, -36},
        {875, -36}
    };

    public static double [][]distalPoints = new double[][]{
        {15906, 2027},
        {15906, 2027},
        {16506, 2384},
        {16506, 2384},
        {17091, 2654},
        {17091, 2654},
        {17682, 2849},
        {17682, 2849},
        {18320, 2987},
        {18320, 2987},
        {19024, 3113},
        {19024, 3113},
        {19730, 3221},
        {19730, 3221},
        {20413, 3318},
        {20413, 3318},
        {21099, 3414},
        {21099, 3414},
        {21757, 3439},
        {21757, 3439},
        {22364, 3347},
        {22364, 3347},
        {22926, 3201},
        {22926, 3201},
        {23423, 3019},
        {23423, 3019},
        {23867, 2780},
        {23867, 2780},
        {24259, 2513},
        {24259, 2513},
        {24608, 2255},
        {24608, 2255},
        {24919, 2005},
        {24919, 2005},
        {25126, 1718},
        {25126, 1718},
        {25177, 1327},
        {25177, 1327},
        {25161, 918},
        {25161, 918},
        {25157, 562},
        {25157, 562},
        {25159, 253},
        {25159, 253},
        {25122, 4},
        {25122, 4},
        {25018, -153},
        {25018, -153},
        {24854, -300},
        {24854, -300},
        {24624, -522},
        {24624, -522},
        {24327, -819},
        {24327, -819},
        {23985, -1125},
        {23985, -1125},
        {23637, -1372},
        {23637, -1372},
        {23292, -1555},
        {23292, -1555},
        {22948, -1673},
        {22948, -1673},
        {22597, -1729},
        {22597, -1729},
        {22267, -1720},
        {22267, -1720},
        {21981, -1659},
        {21981, -1659},
        {21809, -1495},
        {21809, -1495},
        {21809, -1495},
        {21882, -732},
        {21882, -732},
        {21901, -380},
        {21901, -380},
        {21907, -86},
        {21907, -86},
        {21904, 92},
        {21904, 92},
        {21904, 74},
        {21904, 74},
        {21904, 24},
        {21904, 24},
        {21904, 4},
        {21904, 4},
        {21904, 4},
        {21904, -3},
        {21904, -3},
        {21904, 0},
        {21904, 0},
        {21904, 0},
        {21904, 0},
        {21904, 0},
        {21904, 0}
    };

    public static double []wristPoints = new double[]{
        0.344388276,
        0.344388276,
        0.344388276,
        0.344388276,
        0.344388276,
        0.344388276,
        0.344388276,
        0.344388276,
        0.344388276,
        0.344388276,
        0.344388276,
        0.344388276,
        0.344388276,
        0.344388276,
        0.344388276,
        0.338495165,
        0.338495165,
        0.338495165,
        0.338495165,
        0.338495165,
        0.338495165,
        0.338495165,
        0.338495165,
        0.338495165,
        0.338495165,
        0.338495165,
        0.338495165,
        0.338495165,
        0.338495165,
        0.338495165,
        0.338495165,
        0.338495165,
        0.338495165,
        0.338495165,
        0.338495165,
        0.275064379,
        0.275064379,
        0.275064379,
        0.275064379,
        0.275064379,
        0.275064379,
        0.275064379,
        0.275064379,
        0.275064379,
        0.275064379,
        0.275064379,
        0.275064379,
        0.275064379,
        0.275064379,
        0.275064379,
        0.275064379,
        0.275064379,
        0.275064379,
        0.275064379,
        0.275064379,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833,
        0.244921833
    };

};
