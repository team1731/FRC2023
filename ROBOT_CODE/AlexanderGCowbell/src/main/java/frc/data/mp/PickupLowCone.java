package frc.data.mp;

public class PickupLowCone {
    public static ArmPath getArmPath() {
        return new ArmPath(kNumPoints, proximalPoints, distalPoints, kWristFlexIndex, kWristFlexPosition, kWristExtendIndex, kWristMaxVelocity);
    }

    public static final int kNumPoints = 154;
    public static final int kWristFlexIndex = 5;
    public static final double kWristFlexPosition = 0.21;
    public static final int kWristExtendIndex = 150;
    public static final double kWristMaxVelocity = 3000;

    // Position (ticks)	Velocity (RPM)

    public static double [][]proximalPoints = new double[][]{
        {-7452, -966},
        {-7452, -966},
        {-7782, -1158},
        {-7782, -1158},
        {-8555, -1592},
        {-8555, -1592},
        {-8555, -1592},
        {-8555, -1592},
        {-8555, -1592},
        {-8555, -1592},
        {-8555, -1592},
        {-8555, -1592},
        {-8987, -1797},
        {-8987, -1797},
        {-8987, -1797},
        {-8987, -1797},
        {-8987, -1797},
        {-9453, -1994},
        {-9453, -1994},
        {-9968, -2180},
        {-9968, -2180},
        {-10521, -2358},
        {-10521, -2358},
        {-11099, -2538},
        {-11099, -2538},
        {-11673, -2680},
        {-11673, -2680},
        {-12243, -2786},
        {-12243, -2786},
        {-12796, -2825},
        {-12796, -2825},
        {-13344, -2823},
        {-13344, -2823},
        {-13872, -2774},
        {-13872, -2774},
        {-14391, -2722},
        {-14391, -2722},
        {-14890, -2649},
        {-14890, -2649},
        {-15394, -2600},
        {-15394, -2600},
        {-15904, -2560},
        {-15904, -2560},
        {-16430, -2559},
        {-16430, -2559},
        {-16944, -2552},
        {-16944, -2552},
        {-17449, -2561},
        {-17449, -2561},
        {-17937, -2545},
        {-17937, -2545},
        {-18414, -2513},
        {-18414, -2513},
        {-18860, -2433},
        {-18860, -2433},
        {-19287, -2348},
        {-19287, -2348},
        {-19688, -2242},
        {-19688, -2242},
        {-20062, -2129},
        {-20062, -2129},
        {-20400, -1993},
        {-20400, -1993},
        {-20698, -1844},
        {-20698, -1844},
        {-20698, -1844},
        {-20961, -1680},
        {-20961, -1680},
        {-21215, -1533},
        {-21215, -1533},
        {-21461, -1404},
        {-21461, -1404},
        {-21691, -1294},
        {-21691, -1294},
        {-21691, -1294},
        {-22105, -1147},
        {-22105, -1147},
        {-22276, -1065},
        {-22276, -1065},
        {-22414, -958},
        {-22414, -958},
        {-22520, -835},
        {-22520, -835},
        {-22591, -691},
        {-22591, -691},
        {-22657, -557},
        {-22657, -557},
        {-22657, -557},
        {-22689, -417},
        {-22689, -417},
        {-22824, -303},
        {-22824, -303},
        {-22824, -303},
        {-22943, -350},
        {-22943, -350},
        {-23071, -410},
        {-23071, -410},
        {-23194, -502},
        {-23194, -502},
        {-23304, -576},
        {-23304, -576},
        {-23393, -570},
        {-23393, -570},
        {-23444, -505},
        {-23444, -505},
        {-23477, -410},
        {-23477, -410},
        {-23501, -312},
        {-23501, -312},
        {-23492, -193},
        {-23492, -193},
        {-23486, -96},
        {-23486, -96},
        {-23486, -96},
        {-23488, -45},
        {-23488, -45},
        {-23488, -13},
        {-23488, -13},
        {-23488, 13},
        {-23488, 13},
        {-23488, 5},
        {-23488, 5},
        {-23488, -2},
        {-23488, -2},
        {-23488, 0},
        {-23488, 0},
        {-23488, 0},
        {-23488, 0},
        {-23488, 0},
        {-23488, 0},
        {-23488, 0},
        {-23488, 0},
        {-23488, 0},
        {-23488, 0},
        {-23489, -1},
        {-23489, -1},
        {-23491, -3},
        {-23491, -3},
        {-23496, -8},
        {-23496, -8},
        {-23504, -15},
        {-23504, -15},
        {-23523, -35},
        {-23523, -35},
        {-23535, -46},
        {-23535, -46},
        {-23535, -44},
        {-23535, -44},
        {-23535, -39},
        {-23535, -39},
        {-23535, -32},
        {-23535, -32},
        {-23535, -12},
        {-23535, -12}
    };

    public static double [][]distalPoints = new double[][]{
        {13308, 375},
        {13308, 375},
        {13525, 608},
        {13525, 608},
        {14163, 1163},
        {14163, 1163},
        {14163, 1163},
        {14163, 1163},
        {14163, 1163},
        {14163, 1163},
        {14163, 1163},
        {14163, 1163},
        {14601, 1453},
        {14601, 1453},
        {14601, 1453},
        {14601, 1453},
        {14601, 1453},
        {15098, 1778},
        {15098, 1778},
        {15640, 2101},
        {15640, 2101},
        {16240, 2419},
        {16240, 2419},
        {16888, 2713},
        {16888, 2713},
        {17540, 2934},
        {17540, 2934},
        {18175, 3072},
        {18175, 3072},
        {18800, 3159},
        {18800, 3159},
        {19439, 3199},
        {19439, 3199},
        {20091, 3203},
        {20091, 3203},
        {20729, 3190},
        {20729, 3190},
        {21340, 3167},
        {21340, 3167},
        {21923, 3124},
        {21923, 3124},
        {22492, 3055},
        {22492, 3055},
        {23067, 2979},
        {23067, 2979},
        {23624, 2898},
        {23624, 2898},
        {24148, 2811},
        {24148, 2811},
        {24634, 2717},
        {24634, 2717},
        {25093, 2607},
        {25093, 2607},
        {25514, 2454},
        {25514, 2454},
        {25896, 2279},
        {25896, 2279},
        {25896, 2279},
        {26248, 2107},
        {26248, 2107},
        {26580, 1951},
        {26580, 1951},
        {26891, 1804},
        {26891, 1804},
        {27180, 1671},
        {27180, 1671},
        {27682, 1437},
        {27682, 1437},
        {27682, 1437},
        {27920, 1343},
        {27920, 1343},
        {28144, 1257},
        {28144, 1257},
        {28344, 1168},
        {28344, 1168},
        {28522, 1088},
        {28522, 1088},
        {28677, 999},
        {28677, 999},
        {28801, 887},
        {28801, 887},
        {28898, 759},
        {28898, 759},
        {28964, 625},
        {28964, 625},
        {29001, 485},
        {29001, 485},
        {29001, 485},
        {28988, 318},
        {28988, 318},
        {28972, 77},
        {28972, 77},
        {28972, 77},
        {28973, 12},
        {28973, 12},
        {28973, -28},
        {28973, -28},
        {28973, -16},
        {28973, -16},
        {28973, -2},
        {28973, -2},
        {28973, 1},
        {28973, 1},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28973, 0},
        {28974, 1},
        {28974, 1},
        {28976, 3},
        {28976, 3},
        {28977, 4},
        {28977, 4},
        {28976, 3},
        {28976, 3},
        {28976, 3},
        {28976, 3},
        {28976, 2},
        {28976, 2},
        {28976, 0},
        {28976, 0},
        {28976, -1},
        {28976, -1}
    };

    public static double []wristPoints = new double[]{
        0.519007802,
        0.519007802,
        0.519007802,
        0.519007802,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.454666644,
        0.378468186,
        0.378468186,
        0.378468186,
        0.378468186,
        0.378468186,
        0.378468186,
        0.378468186,
        0.378468186,
        0.378468186,
        0.378468186,
        0.378468186,
        0.378468186,
        0.378468186,
        0.378468186,
        0.378468186,
        0.378468186,
        0.378468186,
        0.378468186,
        0.378468186,
        0.378468186,
        0.308329195,
        0.308329195,
        0.308329195,
        0.308329195,
        0.308329195,
        0.308329195,
        0.308329195,
        0.308329195,
        0.308329195,
        0.308329195,
        0.308329195,
        0.308329195,
        0.308329195,
        0.308329195,
        0.308329195,
        0.308329195,
        0.308329195,
        0.308329195,
        0.308329195,
        0.308329195,
        0.253575355,
        0.253575355,
        0.253575355,
        0.253575355,
        0.253575355,
        0.253575355,
        0.253575355,
        0.253575355,
        0.253575355,
        0.253575355,
        0.253575355,
        0.253575355,
        0.253575355,
        0.253575355,
        0.253575355,
        0.253575355,
        0.253575355,
        0.253575355,
        0.253575355,
        0.253575355,
        0.24389258,
        0.24389258,
        0.24389258,
        0.24389258,
        0.24389258,
        0.24389258,
        0.24389258,
        0.24389258,
        0.24389258,
        0.24389258,
        0.24389258,
        0.24389258,
        0.24389258,
        0.24389258,
        0.24389258,
        0.24389258,
        0.24389258,
        0.24389258,
        0.24389258,
        0.24389258,
        0.2351363,
        0.2351363,
        0.2351363,
        0.2351363,
        0.2351363,
        0.2351363,
        0.2351363,
        0.2351363,
        0.2351363,
        0.2351363,
        0.2351363,
        0.2351363,
        0.2351363,
        0.2351363,
        0.2351363,
        0.2351363,
        0.2351363,
        0.2351363,
        0.2351363,
        0.2351363,
        0.238960654,
        0.238960654,
        0.238960654,
        0.238960654,
        0.238960654,
        0.238960654,
        0.238960654,
        0.238960654,
        0.238960654,
        0.238960654,
        0.238960654,
        0.238960654,
        0.238960654,
        0.238960654,
        0.238960654,
        0.238960654,
        0.238960654,
        0.238960654,
        0.238960654,
        0.238960654,
        0.239853233
    };

};
