package frc.data.mp;

public class PickupLowThiefExtend{
    public static ArmPath getArmPath() {
        return new ArmPath(kNumPoints, proximalPoints, distalPoints, kWristFlexIndex, kWristFlexPosition, kWristExtendIndex, kWristMaxVelocity);
    }

    public static final int kNumPoints = 223;
    public static final int kWristFlexIndex = 100;
    public static final double kWristFlexPosition = 0.2;
    public static final int kWristExtendIndex = 222;
    public static final double kWristMaxVelocity = 2000;

    // Position (ticks)	Velocity (RPM)

   
  
    public static double [][]proximalPoints = new double[][]{
        {-9887, -7},
        {-9963, -78},
        {-9963, -78},
        {-10154, -266},
        {-10154, -266},
        {-10154, -266},
        {-10632, -745},
        {-10632, -745},
        {-10845, -952},
        {-10845, -952},
        {-11057, -1092},
        {-11057, -1092},
        {-11281, -1128},
        {-11281, -1128},
        {-11506, -1110},
        {-11506, -1110},
        {-11712, -1081},
        {-11712, -1081},
        {-11899, -1054},
        {-11899, -1054},
        {-11899, -1054},
        {-12055, -1001},
        {-12055, -1001},
        {-12182, -905},
        {-12182, -905},
        {-12278, -777},
        {-12278, -777},
        {-12357, -649},
        {-12357, -649},
        {-12357, -504},
        {-12397, -504},
        {-12397, -504},
        {-12433, -252},
        {-12433, -252},
        {-12502, -225},
        {-12502, -225},
        {-12550, -195},
        {-12550, -195},
        {-12610, -211},
        {-12610, -211},
        {-12686, -278},
        {-12686, -278},
        {-12748, -315},
        {-12748, -315},
        {-12748, -319},
        {-12820, -319},
        {-12876, -325},
        {-12876, -325},
        {-12876, -325},
        {-12954, -344},
        {-12954, -344},
        {-13043, -356},
        {-13043, -356},
        {-13159, -409},
        {-13159, -409},
        {-13314, -489},
        {-13314, -489},
        {-13513, -630},
        {-13513, -630},
        {-13756, -794},
        {-13756, -794},
        {-14043, -993},
        {-14043, -993},
        {-14043, -993},
        {-14714, -1393},
        {-14714, -1393},
        {-15098, -1579},
        {-15098, -1579},
        {-15514, -1753},
        {-15514, -1753},
        {-15514, -1920},
        {-15971, -1920},
        {-15971, -1920},
        {-16463, -2093},
        {-16463, -2093},
        {-16992, -2270},
        {-16992, -2270},
        {-17562, -2456},
        {-17562, -2456},
        {-18174, -2652},
        {-18174, -2652},
        {-18813, -2836},
        {-18813, -2836},
        {-19455, -2986},
        {-19455, -2986},
        {-20062, -3069},
        {-20062, -3069},
        {-20641, -3080},
        {-20641, -3080},
        {-21233, -3061},
        {-21233, -3061},
        {-21869, -3057},
        {-21869, -3057},
        {-22555, -3098},
        {-22555, -3098},
        {-23266, -3198},
        {-23266, -3198},
        {-23997, -3351},
        {-23997, -3351},
        {-24756, -3517},
        {-24756, -3517},
        {-25525, -3651},
        {-25525, -3651},
        {-26309, -3750},
        {-26309, -3750},
        {-27100, -3832},
        {-27100, -3832},
        {-27898, -3897},
        {-27898, -3897},
        {-28686, -3929},
        {-28686, -3929},
        {-29448, -3924},
        {-29448, -3924},
        {-30192, -3885},
        {-30192, -3885},
        {-30947, -3849},
        {-30947, -3849},
        {-31715, -3818},
        {-31715, -3818},
        {-32497, -3810},
        {-32497, -3810},
        {-33310, -3858},
        {-33310, -3858},
        {-34135, -3941},
        {-34135, -3941},
        {-34951, -4004},
        {-34951, -4004},
        {-35744, -4029},
        {-35744, -4029},
        {-36517, -4020},
        {-36517, -4020},
        {-37266, -3959},
        {-37266, -3959},
        {-37986, -3856},
        {-37986, -3856},
        {-38662, -3715},
        {-38662, -3715},
        {-38662, -3715},
        {-39869, -3361},
        {-39869, -3361},
        {-39869, -3361},
        {-40411, -3152},
        {-40411, -3152},
        {-40932, -2952},
        {-40932, -2952},
        {-41467, -2807},
        {-41467, -2807},
        {-42112, -2821},
        {-42112, -2821},
        {-42876, -2996},
        {-42876, -2996},
        {-43669, -3248},
        {-43669, -3248},
        {-44435, -3492},
        {-44435, -3492},
        {-45178, -3706},
        {-45178, -3706},
        {-45896, -3783},
        {-45896, -3783},
        {-46578, -3709},
        {-46578, -3709},
        {-47242, -3581},
        {-47242, -3581},
        {-47921, -3491},
        {-47921, -3491},
        {-49424, -3525},
        {-49424, -3525},
        {-49424, -3525},
        {-49424, -3525},
        {-50213, -3631},
        {-50213, -3631},
        {-50998, -3749},
        {-50998, -3749},
        {-51768, -3844},
        {-51768, -3844},
        {-52515, -3867},
        {-52515, -3867},
        {-53232, -3813},
        {-53232, -3813},
        {-53232, -3813},
        {-54585, -3593},
        {-54585, -3593},
        {-54585, -3593},
        {-55231, -3468},
        {-55231, -3468},
        {-55836, -3326},
        {-55836, -3326},
        {-56427, -3198},
        {-56427, -3198},
        {-57017, -3102},
        {-57017, -3102},
        {-57587, -3006},
        {-57587, -3006},
        {-58126, -2899},
        {-58126, -2899},
        {-58126, -2899},
        {-59113, -2690},
        {-59113, -2690},
        {-59570, -2560},
        {-59570, -2560},
        {-60006, -2424},
        {-60006, -2424},
        {-60416, -2294},
        {-60416, -2294},
        {-60796, -2172},
        {-60796, -2172},
        {-61164, -2056},
        {-61164, -2056},
        {-61164, -1954},
        {-61520, -1954},
        {-61520, -1954},
        {-61873, -1869},
        {-61873, -1869},
        {-62221, -1808},
        {-62221, -1808},
        {-62558, -1764},
        {-62558, -1764},
        {-62558, -1764},
        {-63200, -1682},
        {-63200, -1682},
        {-63496, -1626},
        {-63496, -1626},
        {-63767, -1549}
    };

    public static double [][]distalPoints = new double[][]{
        {12397, -41},
        {12345, -90},
        {12345, -90},
        {12201, -231},
        {12201, -231},
        {11973, -455},
        {11973, -455},
        {11685, -731},
        {11685, -731},
        {11352, -1033},
        {11352, -1033},
        {10984, -1351},
        {10984, -1351},
        {10568, -1623},
        {10568, -1623},
        {10088, -1874},
        {10088, -1874},
        {9536, -2138},
        {9536, -2138},
        {8930, -2412},
        {8930, -2412},
        {8311, -2662},
        {8311, -2662},
        {7671, -2888},
        {7671, -2888},
        {6946, -3133},
        {6946, -3133},
        {6120, -3406},
        {6120, -3406},
        {5260, -3662},
        {5260, -3662},
        {5260, -3662},
        {3535, -4126},
        {3535, -4126},
        {2612, -4326},
        {2612, -4326},
        {1665, -4451},
        {1665, -4451},
        {694, -4559},
        {694, -4559},
        {694, -4559},
        {-291, -4690},
        {-291, -4690},
        {-1241, -4778},
        {-1241, -4778},
        {-2151, -4767},
        {-2151, -4767},
        {-3062, -4729},
        {-3062, -4729},
        {-3962, -4661},
        {-3962, -4661},
        {-4809, -4524},
        {-4809, -4524},
        {-5613, -4377},
        {-5613, -4377},
        {-6363, -4218},
        {-6363, -4218},
        {-7057, -4005},
        {-7057, -4005},
        {-7695, -3743},
        {-7695, -3743},
        {-8274, -3476},
        {-8274, -3476},
        {-8274, -3476},
        {-9268, -2917},
        {-9268, -2917},
        {-9624, -2581},
        {-9624, -2581},
        {-9888, -2207},
        {-9888, -2207},
        {-10096, -1835},
        {-10096, -1835},
        {-10096, -1835},
        {-10242, -1449},
        {-10242, -1449},
        {-10304, -1050},
        {-10304, -1050},
        {-10296, -683},
        {-10296, -683},
        {-10267, -389},
        {-10267, -389},
        {-10240, -152},
        {-10240, -152},
        {-10207, 28},
        {-10207, 28},
        {-10122, 176},
        {-10122, 176},
        {-9929, 358},
        {-9929, 358},
        {-9638, 618},
        {-9638, 618},
        {-9262, 963},
        {-9262, 963},
        {-8819, 1373},
        {-8819, 1373},
        {-8349, 1760},
        {-8349, 1760},
        {-7884, 2036},
        {-7884, 2036},
        {-7451, 2182},
        {-7451, 2182},
        {-7050, 2213},
        {-7050, 2213},
        {-6654, 2166},
        {-6654, 2166},
        {-6222, 2127},
        {-6222, 2127},
        {-5702, 2178},
        {-5702, 2178},
        {-5091, 2352},
        {-5091, 2352},
        {-4386, 2651},
        {-4386, 2651},
        {-3626, 3014},
        {-3626, 3014},
        {-2880, 3331},
        {-2880, 3331},
        {-2178, 3520},
        {-2178, 3520},
        {-1497, 3593},
        {-1497, 3593},
        {-825, 3563},
        {-825, 3563},
        {-184, 3448},
        {-184, 3448},
        {414, 3302},
        {414, 3302},
        {414, 3302},
        {1451, 2956},
        {1451, 2956},
        {1867, 2705},
        {1867, 2705},
        {2101, 2304},
        {2101, 2304},
        {2109, 1719},
        {2109, 1719},
        {2022, 1084},
        {2022, 1084},
        {1970, 538},
        {1970, 538},
        {1925, 74},
        {1925, 74},
        {1856, -237},
        {1856, -237},
        {1856, -237},
        {1762, -345},
        {1762, -345},
        {1615, -403},
        {1615, -403},
        {1434, -530},
        {1434, -530},
        {1268, -655},
        {1268, -655},
        {1099, -753},
        {1099, -753},
        {873, -884},
        {873, -884},
        {590, -1021},
        {590, -1021},
        {296, -1134},
        {296, -1134},
        {32, -1232},
        {32, -1232},
        {-206, -1304},
        {-206, -1304},
        {-443, -1317},
        {-443, -1317},
        {-674, -1266},
        {-674, -1266},
        {-901, -1200},
        {-901, -1200},
        {-1121, -1154},
        {-1121, -1154},
        {-1340, -1134},
        {-1340, -1134},
        {-1560, -1118},
        {-1560, -1118},
        {-1798, -1123},
        {-1798, -1123},
        {-2073, -1169},
        {-2073, -1169},
        {-2073, -1169},
        {-2699, -1355},
        {-2699, -1355},
        {-2999, -1435},
        {-2999, -1435},
        {-3274, -1477},
        {-3274, -1477},
        {-3550, -1477},
        {-3550, -1477},
        {-3823, -1443},
        {-3823, -1443},
        {-3823, -1443},
        {-4090, -1392},
        {-4090, -1392},
        {-4347, -1350},
        {-4347, -1350},
        {-4608, -1333},
        {-4608, -1333},
        {-4870, -1320},
        {-4870, -1320},
        {-5130, -1307},
        {-5130, -1307},
        {-5386, -1297},
        {-5386, -1297},
        {-5645, -1298},
        {-5645, -1298},
        {-5891, -1284},
        {-5891, -1284},
        {-6110, -1243},
        {-6110, -1243},
        {-6298, -1171},
        {-6298, -1171},
        {-6465, -1082},
        {-6465, -1082},
        {-6625, -983},
        {-6625, -983},
        {-6799, -910},
        {-6799, -910},
        {-6992, -883},
        {-6992, -883},
        {-7186, -888},
        {-7186, -888}
    };

    public static double []wristPoints = new double[]{
        0.51804024,
        0.51804024,
        0.51804024,
        0.51804024,
        0.51804024,
        0.51804024,
        0.51804024,
        0.51804024,
        0.51804024,
        0.51804024,
        0.51804024,
        0.519113898,
        0.519113898,
        0.519113898,
        0.519113898,
        0.519113898,
        0.519113898,
        0.519113898,
        0.519113898,
        0.519113898,
        0.519113898,
        0.519113898,
        0.519113898,
        0.519113898,
        0.519113898,
        0.519113898,
        0.519113898,
        0.519113898,
        0.519113898,
        0.519113898,
        0.519113898,
        0.518989563,
        0.518989563,
        0.518989563,
        0.518989563,
        0.518989563,
        0.518989563,
        0.518989563,
        0.518989563,
        0.518989563,
        0.518989563,
        0.518989563,
        0.518989563,
        0.518989563,
        0.518989563,
        0.518989563,
        0.518989563,
        0.518989563,
        0.518989563,
        0.518989563,
        0.518989563,
        0.508377373,
        0.508377373,
        0.508377373,
        0.508377373,
        0.508377373,
        0.508377373,
        0.508377373,
        0.508377373,
        0.508377373,
        0.508377373,
        0.508377373,
        0.508377373,
        0.508377373,
        0.508377373,
        0.508377373,
        0.508377373,
        0.508377373,
        0.508377373,
        0.508377373,
        0.508377373,
        0.45178616,
        0.45178616,
        0.45178616,
        0.45178616,
        0.45178616,
        0.45178616,
        0.45178616,
        0.45178616,
        0.45178616,
        0.45178616,
        0.45178616,
        0.45178616,
        0.45178616,
        0.45178616,
        0.45178616,
        0.45178616,
        0.45178616,
        0.45178616,
        0.45178616,
        0.45178616,
        0.382437944,
        0.382437944,
        0.382437944,
        0.382437944,
        0.382437944,
        0.382437944,
        0.382437944,
        0.382437944,
        0.382437944,
        0.382437944,
        0.382437944,
        0.382437944,
        0.382437944,
        0.382437944,
        0.382437944,
        0.382437944,
        0.382437944,
        0.382437944,
        0.382437944,
        0.382437944,
        0.300524712,
        0.300524712,
        0.300524712,
        0.300524712,
        0.300524712,
        0.300524712,
        0.300524712,
        0.300524712,
        0.300524712,
        0.300524712,
        0.300524712,
        0.300524712,
        0.300524712,
        0.300524712,
        0.300524712,
        0.300524712,
        0.300524712,
        0.300524712,
        0.300524712,
        0.300524712,
        0.219564855,
        0.219564855,
        0.219564855,
        0.219564855,
        0.219564855,
        0.219564855,
        0.219564855,
        0.219564855,
        0.219564855,
        0.219564855,
        0.219564855,
        0.219564855,
        0.219564855,
        0.219564855,
        0.219564855,
        0.219564855,
        0.219564855,
        0.219564855,
        0.219564855,
        0.219564855,
        0.193260372,
        0.193260372,
        0.193260372,
        0.193260372,
        0.193260372,
        0.193260372,
        0.193260372,
        0.193260372,
        0.193260372,
        0.193260372,
        0.193260372,
        0.193260372,
        0.193260372,
        0.193260372,
        0.193260372,
        0.193260372,
        0.193260372,
        0.193260372,
        0.193260372,
        0.193260372,
        0.198091805,
        0.198091805,
        0.198091805,
        0.198091805,
        0.198091805,
        0.198091805,
        0.198091805,
        0.198091805,
        0.198091805,
        0.198091805,
        0.198091805,
        0.198091805,
        0.198091805,
        0.198091805,
        0.198091805,
        0.198091805,
        0.198091805,
        0.198091805,
        0.198091805,
        0.198091805,
        0.200114429,
        0.200114429,
        0.200114429,
        0.200114429,
        0.200114429,
        0.200114429,
        0.200114429,
        0.200114429,
        0.200114429,
        0.200114429,
        0.200114429,
        0.200114429,
        0.200114429,
        0.200114429,
        0.200114429,
        0.200114429,
        0.200114429,
        0.200114429,
        0.200114429,
        0.200114429,
        0.200041652,
        0.200041652,
        0.200041652,
        0.200041652,
        0.200041652,
        0.200041652,
        0.200041652,
        0.200041652,
        0.200041652,
        0.200041652,
        0.200041652,
        0.200041652
    };

};
