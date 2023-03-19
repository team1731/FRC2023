package frc.data.mp;

public class ScoreHighCone {
    public static ArmPath getArmPath() {
        return new ArmPath(kNumPoints, proximalPoints, distalPoints, kWristFlexIndex, kWristFlexPosition, kWristExtendIndex, kWristMaxVelocity);
    }

    public static final int kNumPoints = 143;
    public static final int kWristFlexIndex = 50;
    public static final double kWristFlexPosition = 0.3784;
    public static final int kWristExtendIndex = 142;
    public static final double kWristMaxVelocity = 2000;

    // Position (ticks)	Velocity (RPM)

    public static double [][]proximalPoints = new double[][]{
        {-6520, 2},
        {-6525, -4},
        {-6525, -4},
        {-6526, -5},
        {-6526, -5},
        {-6526, -6},
        {-6526, -6},
        {-6526, -6},
        {-6526, -6},
        {-6526, -6},
        {-6526, -6},
        {-6526, -1},
        {-6526, -1},
        {-6526, 0},
        {-6526, 0},
        {-6560, -31},
        {-6560, -31},
        {-6858, -319},
        {-6858, -319},
        {-7432, -890},
        {-7432, -890},
        {-8061, -1521},
        {-8061, -1521},
        {-8641, -2100},
        {-8641, -2100},
        {-8641, -2100},
        {-9657, -2800},
        {-9657, -2800},
        {-10096, -2670},
        {-10096, -2670},
        {-10492, -2435},
        {-10492, -2435},
        {-10853, -2218},
        {-10853, -2218},
        {-11207, -2033},
        {-11207, -2033},
        {-11566, -1912},
        {-11566, -1912},
        {-11931, -1835},
        {-11931, -1835},
        {-12270, -1780},
        {-12270, -1780},
        {-12581, -1729},
        {-12581, -1729},
        {-12891, -1686},
        {-12891, -1686},
        {-13240, -1673},
        {-13240, -1673},
        {-13629, -1697},
        {-13629, -1697},
        {-14060, -1785},
        {-14060, -1785},
        {-14526, -1942},
        {-14526, -1942},
        {-14526, -1942},
        {-15053, -2156},
        {-15053, -2156},
        {-15644, -2399},
        {-15644, -2399},
        {-16317, -2679},
        {-16317, -2679},
        {-17063, -2995},
        {-17063, -2995},
        {-17875, -3338},
        {-17875, -3338},
        {-18739, -3678},
        {-18739, -3678},
        {-18739, -3678},
        {-20639, -4316},
        {-20639, -4316},
        {-21690, -4622},
        {-21690, -4622},
        {-22798, -4918},
        {-22798, -4918},
        {-23928, -5180},
        {-23928, -5180},
        {-25023, -5359},
        {-25023, -5359},
        {-26092, -5451},
        {-26092, -5451},
        {-27206, -5510},
        {-27206, -5510},
        {-27206, -5510},
        {-28345, -5544},
        {-28345, -5544},
        {-29481, -5552},
        {-29481, -5552},
        {-30578, -5555},
        {-30578, -5555},
        {-31642, -5549},
        {-31642, -5549},
        {-32688, -5486},
        {-32688, -5486},
        {-33686, -5346},
        {-33686, -5346},
        {-34624, -5152},
        {-34624, -5152},
        {-35504, -4935},
        {-35504, -4935},
        {-36331, -4697},
        {-36331, -4697},
        {-37106, -4426},
        {-37106, -4426},
        {-37106, -4426},
        {-38494, -3875},
        {-38494, -3875},
        {-39096, -3598},
        {-39096, -3598},
        {-39621, -3297},
        {-39621, -3297},
        {-39621, -3297},
        {-40076, -2979},
        {-40076, -2979},
        {-40465, -2645},
        {-40465, -2645},
        {-40803, -2319},
        {-40803, -2319},
        {-41097, -2009},
        {-41097, -2009},
        {-41352, -1738},
        {-41352, -1738},
        {-41575, -1505},
        {-41575, -1505},
        {-41765, -1305},
        {-41765, -1305},
        {-41922, -1123},
        {-41922, -1123},
        {-42047, -954},
        {-42047, -954},
        {-42147, -800},
        {-42147, -800},
        {-42220, -648},
        {-42220, -648},
        {-42294, -376},
        {-42294, -376},
        {-42294, -376},
        {-42310, -265},
        {-42310, -265},
        {-42330, -185},
        {-42330, -185},
        {-42348, -129},
        {-42348, -129},
        {-42348, -129}
    };

    public static double [][]distalPoints = new double[][]{
        {12127, -4},
        {12075, -53},
        {12075, -53},
        {11897, -228},
        {11897, -228},
        {11631, -493},
        {11631, -493},
        {11345, -779},
        {11345, -779},
        {11029, -1089},
        {11029, -1089},
        {10635, -1432},
        {10635, -1432},
        {10156, -1732},
        {10156, -1732},
        {10156, -1732},
        {8985, -2349},
        {8985, -2349},
        {8150, -2865},
        {8150, -2865},
        {7106, -3510},
        {7106, -3510},
        {5935, -4206},
        {5935, -4206},
        {4751, -4859},
        {4751, -4859},
        {3543, -5429},
        {3543, -5429},
        {2230, -5905},
        {2230, -5905},
        {785, -6313},
        {785, -6313},
        {-764, -6689},
        {-764, -6689},
        {-2372, -7114},
        {-2372, -7114},
        {-4026, -7552},
        {-4026, -7552},
        {-5759, -7983},
        {-5759, -7983},
        {-7572, -8351},
        {-7572, -8351},
        {-9429, -8659},
        {-9429, -8659},
        {-11293, -8914},
        {-11293, -8914},
        {-13156, -9130},
        {-13156, -9130},
        {-15036, -9273},
        {-15036, -9273},
        {-16911, -9335},
        {-16911, -9335},
        {-18772, -9343},
        {-18772, -9343},
        {-18772, -9343},
        {-20604, -9313},
        {-20604, -9313},
        {-22386, -9230},
        {-22386, -9230},
        {-24135, -9100},
        {-24135, -9100},
        {-25843, -8935},
        {-25843, -8935},
        {-27497, -8729},
        {-27497, -8729},
        {-29097, -8495},
        {-29097, -8495},
        {-30633, -8254},
        {-30633, -8254},
        {-32107, -7982},
        {-32107, -7982},
        {-33529, -7697},
        {-33529, -7697},
        {-34905, -7413},
        {-34905, -7413},
        {-36227, -7138},
        {-36227, -7138},
        {-37490, -6866},
        {-37490, -6866},
        {-38720, -6619},
        {-38720, -6619},
        {-39895, -6371},
        {-39895, -6371},
        {-41030, -6131},
        {-41030, -6131},
        {-42105, -5884},
        {-42105, -5884},
        {-43121, -5639},
        {-43121, -5639},
        {-44076, -5362},
        {-44076, -5362},
        {-44975, -5086},
        {-44975, -5086},
        {-45796, -4776},
        {-45796, -4776},
        {-46462, -4372},
        {-46462, -4372},
        {-46892, -3786},
        {-46892, -3786},
        {-47182, -3125},
        {-47182, -3125},
        {-47421, -2463},
        {-47421, -2463},
        {-47421, -2463},
        {-47719, -1270},
        {-47719, -1270},
        {-47702, -821},
        {-47702, -821},
        {-47649, -474},
        {-47649, -474},
        {-47649, -474},
        {-47629, -215},
        {-47629, -215},
        {-47581, 21},
        {-47581, 21},
        {-47503, 212},
        {-47503, 212},
        {-47458, 245},
        {-47458, 245},
        {-47406, 243},
        {-47406, 243},
        {-47343, 286},
        {-47343, 286},
        {-47298, 284},
        {-47298, 284},
        {-47226, 277},
        {-47226, 277},
        {-47168, 290},
        {-47168, 290},
        {-47144, 263},
        {-47144, 263},
        {-47124, 219},
        {-47124, 219},
        {-47066, 162},
        {-47066, 162},
        {-47066, 162},
        {-47073, 96},
        {-47073, 96},
        {-47082, 62},
        {-47082, 62},
        {-47090, 35},
        {-47090, 35},
        {-47090, 35}
    };

    public static double []wristPoints = new double[]{
        0.552163124,
        0.552163124,
        0.552163124,
        0.552163124,
        0.552163124,
        0.552163124,
        0.552163124,
        0.552163124,
        0.552163124,
        0.552128434,
        0.552128434,
        0.552128434,
        0.552128434,
        0.552128434,
        0.552128434,
        0.552128434,
        0.552128434,
        0.552128434,
        0.552128434,
        0.552128434,
        0.552128434,
        0.552128434,
        0.552128434,
        0.552128434,
        0.552128434,
        0.552128434,
        0.552128434,
        0.552128434,
        0.552128434,
        0.546320796,
        0.546320796,
        0.546320796,
        0.546320796,
        0.546320796,
        0.546320796,
        0.546320796,
        0.546320796,
        0.546320796,
        0.546320796,
        0.546320796,
        0.546320796,
        0.546320796,
        0.546320796,
        0.546320796,
        0.546320796,
        0.546320796,
        0.546320796,
        0.546320796,
        0.546320796,
        0.536533713,
        0.536533713,
        0.536533713,
        0.536533713,
        0.536533713,
        0.536533713,
        0.536533713,
        0.536533713,
        0.536533713,
        0.536533713,
        0.536533713,
        0.536533713,
        0.536533713,
        0.536533713,
        0.536533713,
        0.536533713,
        0.536533713,
        0.536533713,
        0.536533713,
        0.536533713,
        0.506282449,
        0.506282449,
        0.506282449,
        0.506282449,
        0.506282449,
        0.506282449,
        0.506282449,
        0.506282449,
        0.506282449,
        0.506282449,
        0.506282449,
        0.506282449,
        0.506282449,
        0.506282449,
        0.506282449,
        0.506282449,
        0.506282449,
        0.506282449,
        0.506282449,
        0.506282449,
        0.443802655,
        0.443802655,
        0.443802655,
        0.443802655,
        0.443802655,
        0.443802655,
        0.443802655,
        0.443802655,
        0.443802655,
        0.443802655,
        0.443802655,
        0.443802655,
        0.443802655,
        0.443802655,
        0.443802655,
        0.443802655,
        0.443802655,
        0.443802655,
        0.443802655,
        0.443802655,
        0.381339073,
        0.381339073,
        0.381339073,
        0.381339073,
        0.381339073,
        0.381339073,
        0.381339073,
        0.381339073,
        0.381339073,
        0.381339073,
        0.381339073,
        0.381339073,
        0.381339073,
        0.381339073,
        0.381339073,
        0.381339073,
        0.381339073,
        0.381339073,
        0.381339073,
        0.381339073,
        0.378435194,
        0.378435194,
        0.378435194,
        0.378435194,
        0.378435194,
        0.378435194,
        0.378435194,
        0.378435194,
        0.378435194,
        0.378435194,
        0.378435194,
        0.378435194,
        0.378435194,
        0.378435194
    };

};
