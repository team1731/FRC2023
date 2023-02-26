package frc.data.mp;

public class ScoreLowCube {
    public static ArmPath getArmPath() {
        return new ArmPath(kNumPoints, proximalPoints, distalPoints, kWristFlexIndex, kWristFlexPosition, kWristExtendIndex, kWristMaxVelocity);
    }

    public static final int kNumPoints = 111;
    public static final int kWristFlexIndex = 75;
    public static final double kWristFlexPosition = 0.42;
    public static final int kWristExtendIndex = 106;
    public static final double kWristMaxVelocity = 3000;

    // Position (ticks)	Velocity (RPM)

    public static double [][]proximalPoints = new double[][]{
        {-5158, 107},
        {-5158, 107},
        {-5160, 6},
        {-5160, 6},
        {-5160, -2},
        {-5160, -2},
        {-5160, -2},
        {-5160, -2},
        {-5160, -2},
        {-5160, -2},
        {-5161, -3},
        {-5161, -3},
        {-5161, -3},
        {-5161, -3},
        {-5162, -4},
        {-5162, -4},
        {-5162, -4},
        {-5176, -16},
        {-5176, -16},
        {-5188, -28},
        {-5188, -28},
        {-5186, -25},
        {-5186, -25},
        {-5186, -24},
        {-5186, -24},
        {-5187, -21},
        {-5187, -21},
        {-5187, -11},
        {-5187, -11},
        {-5187, 1},
        {-5187, 1},
        {-5187, -1},
        {-5187, -1},
        {-5187, -1},
        {-5187, -1},
        {-5187, 0},
        {-5187, 0},
        {-5187, 0},
        {-5187, 0},
        {-5187, 0},
        {-5187, 0},
        {-5187, 0},
        {-5187, 0},
        {-5188, -1},
        {-5188, -1},
        {-5191, -4},
        {-5191, -4},
        {-5192, -5},
        {-5192, -5},
        {-5194, -7},
        {-5194, -7},
        {-5200, -13},
        {-5200, -13},
        {-5223, -33},
        {-5223, -33},
        {-5251, -60},
        {-5251, -60},
        {-5241, -50},
        {-5241, -50},
        {-5237, -42},
        {-5237, -42},
        {-5238, -38},
        {-5238, -38},
        {-5237, -16},
        {-5237, -16},
        {-5237, 14},
        {-5237, 14},
        {-5237, 5},
        {-5237, 5},
        {-5237, 5},
        {-5237, -1},
        {-5237, -1},
        {-5237, 1},
        {-5237, 1},
        {-5237, 0},
        {-5237, 0},
        {-5237, 0},
        {-5237, 0},
        {-5237, 0},
        {-5237, 0},
        {-5237, 0},
        {-5237, 0},
        {-5237, 0},
        {-5237, 0},
        {-5237, 0},
        {-5237, 0},
        {-5237, 0},
        {-5237, 0},
        {-5237, 0},
        {-5237, 0},
        {-5237, 0},
        {-5237, 0},
        {-5234, 3},
        {-5234, 3},
        {-5229, 8},
        {-5229, 8},
        {-5218, 18},
        {-5218, 18},
        {-5192, 44},
        {-5192, 44},
        {-5178, 59},
        {-5178, 59},
        {-5189, 46},
        {-5189, 46},
        {-5188, 41},
        {-5188, 41},
        {-5188, 31},
        {-5188, 31},
        {-5188, 5},
        {-5188, 5},
        {-5188, -10}
    };

    public static double [][]distalPoints = new double[][]{
        {10228, -1138},
        {10228, -1138},
        {9885, -1373},
        {9885, -1373},
        {9054, -1772},
        {9054, -1772},
        {9054, -1772},
        {9054, -1772},
        {9054, -1772},
        {9054, -1772},
        {8538, -1998},
        {8538, -1998},
        {8538, -1998},
        {8538, -1998},
        {7940, -2275},
        {7940, -2275},
        {7940, -2275},
        {6635, -2854},
        {6635, -2854},
        {5961, -3084},
        {5961, -3084},
        {5261, -3270},
        {5261, -3270},
        {4537, -3401},
        {4537, -3401},
        {3797, -3493},
        {3797, -3493},
        {3083, -3551},
        {3083, -3551},
        {2387, -3573},
        {2387, -3573},
        {1675, -3586},
        {1675, -3586},
        {942, -3595},
        {942, -3595},
        {196, -3601},
        {196, -3601},
        {-536, -3618},
        {-536, -3618},
        {-1252, -3639},
        {-1252, -3639},
        {-1961, -3635},
        {-1961, -3635},
        {-2660, -3603},
        {-2660, -3603},
        {-3362, -3560},
        {-3362, -3560},
        {-4102, -3564},
        {-4102, -3564},
        {-4908, -3651},
        {-4908, -3651},
        {-5765, -3797},
        {-5765, -3797},
        {-6661, -3992},
        {-6661, -3992},
        {-7566, -4198},
        {-7566, -4198},
        {-8472, -4365},
        {-8472, -4365},
        {-9345, -4437},
        {-9345, -4437},
        {-10175, -4415},
        {-10175, -4415},
        {-10982, -4327},
        {-10982, -4327},
        {-11760, -4198},
        {-11760, -4198},
        {-12497, -4032},
        {-12497, -4032},
        {-12497, -4032},
        {-13185, -3847},
        {-13185, -3847},
        {-13824, -3655},
        {-13824, -3655},
        {-14422, -3448},
        {-14422, -3448},
        {-14977, -3226},
        {-14977, -3226},
        {-15495, -3004},
        {-15495, -3004},
        {-15992, -2812},
        {-15992, -2812},
        {-16487, -2669},
        {-16487, -2669},
        {-16947, -2530},
        {-16947, -2530},
        {-17359, -2388},
        {-17359, -2388},
        {-17722, -2235},
        {-17722, -2235},
        {-18046, -2062},
        {-18046, -2062},
        {-18332, -1852},
        {-18332, -1852},
        {-18571, -1631},
        {-18571, -1631},
        {-18777, -1425},
        {-18777, -1425},
        {-18952, -1237},
        {-18952, -1237},
        {-19097, -1058},
        {-19097, -1058},
        {-19214, -889},
        {-19214, -889},
        {-19286, -722},
        {-19286, -722},
        {-19346, -574},
        {-19346, -574},
        {-19365, -419},
        {-19365, -419},
        {-19350, -259}
    };

    public static double []wristPoints = new double[]{
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        0.5
    };

};