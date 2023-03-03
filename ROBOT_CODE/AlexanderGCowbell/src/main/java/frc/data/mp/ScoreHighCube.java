package frc.data.mp;

public class ScoreHighCube {
    public static ArmPath getArmPath() {
        return new ArmPath(kNumPoints, proximalPoints, distalPoints, kWristFlexIndex, kWristFlexPosition, kWristExtendIndex, kWristMaxVelocity);
    }

    public static final int kNumPoints = 190;
    public static final int kWristFlexIndex = 75;
    public static final double kWristFlexPosition = 0.4507;
    public static final int kWristExtendIndex = 185;
    public static final double kWristMaxVelocity = 2000;

    // Position (ticks)	Velocity (RPM)

    public static double [][]proximalPoints = new double[][]{
        {1752, 0},
        {1752, 0},
        {1752, 0},
        {1752, 0},
        {1752, 0},
        {1752, 0},
        {1752, 0},
        {1752, 0},
        {1752, 0},
        {1752, 0},
        {1752, 0},
        {1752, 0},
        {1752, 0},
        {1752, 0},
        {1752, 0},
        {1752, 0},
        {1752, 0},
        {1752, 0},
        {1751, -1},
        {1751, -1},
        {1751, -1},
        {1751, -1},
        {1750, -2},
        {1750, -2},
        {1750, -2},
        {1750, -2},
        {1748, -3},
        {1748, -3},
        {1748, -3},
        {1748, -3},
        {1748, -3},
        {1747, -3},
        {1747, -3},
        {1746, -4},
        {1746, -4},
        {1744, -5},
        {1740, -8},
        {1740, -8},
        {1740, -8},
        {1725, -22},
        {1725, -22},
        {1665, -78},
        {1665, -78},
        {1523, -217},
        {1523, -217},
        {1326, -410},
        {1326, -410},
        {1083, -646},
        {1083, -646},
        {797, -918},
        {797, -918},
        {452, -1202},
        {452, -1202},
        {35, -1478},
        {35, -1478},
        {-439, -1754},
        {-439, -1754},
        {-960, -2034},
        {-960, -2034},
        {-1521, -2307},
        {-1521, -2307},
        {-2118, -2563},
        {-2118, -2563},
        {-2740, -2767},
        {-2740, -2767},
        {-3382, -2938},
        {-3382, -2938},
        {-4033, -3069},
        {-4033, -3069},
        {-4700, -3176},
        {-4700, -3176},
        {-5398, -3275},
        {-5398, -3275},
        {-6110, -3366},
        {-6110, -3366},
        {-6831, -3445},
        {-6831, -3445},
        {-7556, -3522},
        {-7556, -3522},
        {-8286, -3582},
        {-8286, -3582},
        {-9023, -3623},
        {-9023, -3623},
        {-9776, -3666},
        {-9776, -3666},
        {-10566, -3734},
        {-10566, -3734},
        {-11382, -3820},
        {-11382, -3820},
        {-12210, -3922},
        {-12210, -3922},
        {-13039, -4012},
        {-13039, -4012},
        {-13887, -4107},
        {-13887, -4107},
        {-14791, -4221},
        {-14791, -4221},
        {-15739, -4353},
        {-15739, -4353},
        {-15739, -4353},
        {-16709, -4494},
        {-16709, -4494},
        {-17686, -4644},
        {-17686, -4644},
        {-18651, -4760},
        {-18651, -4760},
        {-19619, -4823},
        {-19619, -4823},
        {-19619, -4823},
        {-21607, -4897},
        {-21607, -4897},
        {-22580, -4895},
        {-22580, -4895},
        {-23508, -4858},
        {-23508, -4858},
        {-24386, -4773},
        {-24386, -4773},
        {-25228, -4628},
        {-25228, -4628},
        {-26054, -4455},
        {-26054, -4455},
        {-26868, -4294},
        {-26868, -4294},
        {-27639, -4139},
        {-27639, -4139},
        {-28355, -3976},
        {-28355, -3976},
        {-29037, -3816},
        {-29037, -3816},
        {-29692, -3642},
        {-29692, -3642},
        {-30323, -3460},
        {-30323, -3460},
        {-30940, -3306},
        {-30940, -3306},
        {-31532, -3181},
        {-31532, -3181},
        {-31532, -3181},
        {-32103, -3071},
        {-32103, -3071},
        {-32642, -2956},
        {-32642, -2956},
        {-33157, -2840},
        {-33157, -2840},
        {-33666, -2729},
        {-33666, -2729},
        {-34178, -2649},
        {-34178, -2649},
        {-34682, -2581},
        {-34682, -2581},
        {-35171, -2530},
        {-35171, -2530},
        {-35639, -2483},
        {-35639, -2483},
        {-36088, -2425},
        {-36088, -2425},
        {-36913, -2234},
        {-36913, -2234},
        {-36913, -2234},
        {-37335, -2167},
        {-37335, -2167},
        {-37766, -2128},
        {-37766, -2128},
        {-38189, -2101},
        {-38189, -2101},
        {-38586, -2085},
        {-38586, -2085},
        {-38586, -2085},
        {-38962, -2050},
        {-38962, -2050},
        {-39299, -1967},
        {-39299, -1967},
        {-39599, -1837},
        {-39599, -1837},
        {-39877, -1694},
        {-39877, -1694},
        {-40141, -1561},
        {-40141, -1561},
        {-40401, -1442},
        {-40401, -1442},
        {-40401, -1442},
        {-40926, -1329},
        {-40926, -1329},
        {-41169, -1293},
        {-41169, -1293},
        {-41391, -1251},
        {-41391, -1251},
        {-41585, -1189},
        {-41585, -1189},
        {-41750, -1091}
    };

    public static double [][]distalPoints = new double[][]{
        {3422, -7},
        {3422, -7},
        {3395, -33},
        {3395, -33},
        {3357, -71},
        {3357, -71},
        {3263, -162},
        {3263, -162},
        {3125, -298},
        {3125, -298},
        {2933, -481},
        {2933, -481},
        {2677, -709},
        {2677, -709},
        {2336, -1008},
        {2336, -1008},
        {1902, -1347},
        {1902, -1347},
        {1378, -1731},
        {1378, -1731},
        {772, -2146},
        {772, -2146},
        {78, -2582},
        {78, -2582},
        {-688, -3008},
        {-688, -3008},
        {-1513, -3402},
        {-1513, -3402},
        {-1513, -3402},
        {-3258, -4020},
        {-3258, -4020},
        {-4185, -4254},
        {-4185, -4254},
        {-5180, -4483},
        {-5180, -4483},
        {-6208, -4689},
        {-7264, -4875},
        {-7264, -4875},
        {-7264, -4875},
        {-8336, -5072},
        {-8336, -5072},
        {-9414, -5225},
        {-9414, -5225},
        {-10502, -5321},
        {-10502, -5321},
        {-11578, -5368},
        {-11578, -5368},
        {-12601, -5342},
        {-12601, -5342},
        {-13560, -5228},
        {-13560, -5228},
        {-14459, -5054},
        {-14459, -5054},
        {-15290, -4799},
        {-15290, -4799},
        {-16054, -4488},
        {-16054, -4488},
        {-16767, -4177},
        {-16767, -4177},
        {-17416, -3869},
        {-17416, -3869},
        {-18004, -3555},
        {-18004, -3555},
        {-18536, -3257},
        {-18536, -3257},
        {-19019, -2975},
        {-19019, -2975},
        {-19462, -2704},
        {-19462, -2704},
        {-19939, -2526},
        {-19939, -2526},
        {-20523, -2516},
        {-20523, -2516},
        {-21163, -2620},
        {-21163, -2620},
        {-21772, -2748},
        {-21772, -2748},
        {-22368, -2899},
        {-22368, -2899},
        {-22977, -3034},
        {-22977, -3034},
        {-23562, -3042},
        {-23562, -3042},
        {-24096, -2939},
        {-24096, -2939},
        {-24578, -2811},
        {-24578, -2811},
        {-25011, -2651},
        {-25011, -2651},
        {-25406, -2439},
        {-25406, -2439},
        {-25767, -2212},
        {-25767, -2212},
        {-26125, -2034},
        {-26125, -2034},
        {-26477, -1903},
        {-26477, -1903},
        {-26791, -1785},
        {-26791, -1785},
        {-26791, -1785},
        {-27072, -1669},
        {-27072, -1669},
        {-27320, -1558},
        {-27320, -1558},
        {-27539, -1420},
        {-27539, -1420},
        {-27740, -1268},
        {-27740, -1268},
        {-27740, -1268},
        {-28145, -1075},
        {-28145, -1075},
        {-28391, -1070},
        {-28391, -1070},
        {-28681, -1139},
        {-28681, -1139},
        {-28681, -1139},
        {-29006, -1261},
        {-29006, -1261},
        {-29328, -1387},
        {-29328, -1387},
        {-29641, -1493},
        {-29641, -1493},
        {-29641, -1493},
        {-30328, -1643},
        {-30328, -1643},
        {-30736, -1728},
        {-30736, -1728},
        {-31172, -1840},
        {-31172, -1840},
        {-31628, -1980},
        {-31628, -1980},
        {-32089, -2119},
        {-32089, -2119},
        {-32543, -2214},
        {-32543, -2214},
        {-32986, -2249},
        {-32986, -2249},
        {-32986, -2249},
        {-33421, -2248},
        {-33421, -2248},
        {-33858, -2231},
        {-33858, -2231},
        {-34309, -2219},
        {-34309, -2219},
        {-34795, -2249},
        {-34795, -2249},
        {-35285, -2298},
        {-35285, -2298},
        {-35739, -2320},
        {-35739, -2320},
        {-36150, -2295},
        {-36150, -2295},
        {-36527, -2223},
        {-36527, -2223},
        {-36886, -2095},
        {-36886, -2095},
        {-37259, -1978},
        {-37259, -1978},
        {-37259, -1978},
        {-37956, -1808},
        {-37956, -1808},
        {-38232, -1709},
        {-38232, -1709},
        {-38460, -1582},
        {-38460, -1582},
        {-38642, -1391},
        {-38642, -1391},
        {-38642, -1391},
        {-38776, -1158},
        {-38776, -1158},
        {-38826, -882},
        {-38826, -882},
        {-38815, -592},
        {-38815, -592},
        {-38818, -365},
        {-38818, -365},
        {-38816, -179},
        {-38816, -179},
        {-38816, -43},
        {-38816, -43},
        {-38816, -43},
        {-38822, -7},
        {-38822, -7},
        {-38822, -4},
        {-38822, -4},
        {-38822, -6},
        {-38822, -6},
        {-38822, -6},
        {-38822, -6},
        {-38822, -4}
    };

    public static double []wristPoints = new double[]{
        0.549349904,
        0.549349904,
        0.549349904,
        0.549349904,
        0.549349904,
        0.549349904,
        0.549349904,
        0.549349904,
        0.549349904,
        0.549349904,
        0.549349904,
        0.549349904,
        0.549349904,
        0.549349904,
        0.549349904,
        0.549349904,
        0.549349904,
        0.549349904,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550418973,
        0.550418973,
        0.550418973,
        0.550418973,
        0.550418973,
        0.550418973,
        0.550418973,
        0.550418973,
        0.550418973,
        0.550418973,
        0.550418973,
        0.550418973,
        0.550418973,
        0.550418973,
        0.550418973,
        0.550418973,
        0.550418973,
        0.550418973,
        0.550418973,
        0.550418973,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550209165,
        0.550209165,
        0.550209165,
        0.550209165,
        0.550209165,
        0.550209165,
        0.550209165,
        0.550209165,
        0.550209165,
        0.550209165,
        0.550209165,
        0.550209165,
        0.550209165,
        0.550209165,
        0.550209165,
        0.550209165,
        0.550209165,
        0.550209165,
        0.550209165,
        0.550209165,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550313234,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550314784,
        0.550206065,
        0.550206065,
        0.550206065,
        0.550206065,
        0.550206065,
        0.550206065,
        0.550206065,
        0.550206065,
        0.550206065,
        0.550206065,
        0.550206065,
        0.550206065
    };

};
