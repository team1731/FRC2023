package frc.data.mp;

public class ScoreMediumCube {
    public static ArmPath getArmPath() {
        return new ArmPath(kNumPoints, proximalPoints, distalPoints, kWristFlexIndex, kWristFlexPosition, kWristExtendIndex, kWristMaxVelocity);
    }

    public static final int kNumPoints = 217;
    public static final int kWristFlexIndex = 75;
    public static final double kWristFlexPosition = 0.551;
    public static final int kWristExtendIndex = 112;
    public static final double kWristMaxVelocity = 2000;

    // Position (ticks)	Velocity (RPM)

    public static double [][]proximalPoints = new double[][]{
        {-2347, 0},
        {-2347, 0},
        {-2347, 0},
        {-2347, 0},
        {-2347, 0},
        {-2347, 0},
        {-2347, 0},
        {-2347, 0},
        {-2347, 0},
        {-2347, 0},
        {-2347, 0},
        {-2347, 0},
        {-2347, 0},
        {-2347, 0},
        {-2347, 0},
        {-2347, 0},
        {-2347, 0},
        {-2347, 0},
        {-2347, 0},
        {-2347, 0},
        {-2347, 0},
        {-2347, 0},
        {-2347, 0},
        {-2350, -2},
        {-2350, -2},
        {-2359, -12},
        {-2359, -12},
        {-2378, -30},
        {-2378, -30},
        {-2417, -69},
        {-2417, -69},
        {-2424, -77},
        {-2424, -77},
        {-2424, -75},
        {-2424, -75},
        {-2425, -66},
        {-2425, -66},
        {-2429, -51},
        {-2429, -51},
        {-2430, -14},
        {-2430, -14},
        {-2422, 2},
        {-2422, 2},
        {-2401, 22},
        {-2401, 22},
        {-2399, 26},
        {-2399, 26},
        {-2400, 28},
        {-2400, 28},
        {-2409, 21},
        {-2409, 21},
        {-2412, 10},
        {-2412, 10},
        {-2414, -12},
        {-2414, -12},
        {-2414, -12},
        {-2439, -38},
        {-2439, -38},
        {-2570, -163},
        {-2570, -163},
        {-2799, -379},
        {-2799, -379},
        {-3071, -649},
        {-3071, -649},
        {-3347, -923},
        {-3347, -923},
        {-3645, -1196},
        {-3645, -1196},
        {-4001, -1422},
        {-4001, -1422},
        {-4410, -1605},
        {-4410, -1605},
        {-4867, -1787},
        {-4867, -1787},
        {-5352, -1997},
        {-5352, -1997},
        {-5858, -2204},
        {-5858, -2204},
        {-6394, -2388},
        {-6394, -2388},
        {-6966, -2550},
        {-6966, -2550},
        {-7555, -2684},
        {-7555, -2684},
        {-8152, -2795},
        {-8152, -2795},
        {-8749, -2887},
        {-8749, -2887},
        {-9355, -2957},
        {-9355, -2957},
        {-9963, -2995},
        {-9963, -2995},
        {-10582, -3026},
        {-10582, -3026},
        {-11196, -3044},
        {-11196, -3044},
        {-11784, -3038},
        {-11784, -3038},
        {-12357, -3006},
        {-12357, -3006},
        {-12901, -2941},
        {-12901, -2941},
        {-13435, -2857},
        {-13435, -2857},
        {-13961, -2768},
        {-13961, -2768},
        {-14499, -2717},
        {-14499, -2717},
        {-15023, -2667},
        {-15023, -2667},
        {-15525, -2627},
        {-15525, -2627},
        {-15994, -2560},
        {-15994, -2560},
        {-16455, -2496},
        {-16455, -2496},
        {-16914, -2417},
        {-16914, -2417},
        {-17384, -2362},
        {-17384, -2362},
        {-17842, -2318},
        {-17842, -2318},
        {-18293, -2301},
        {-18293, -2301},
        {-18729, -2275},
        {-18729, -2275},
        {-19172, -2258},
        {-19172, -2258},
        {-19620, -2238},
        {-19620, -2238},
        {-20057, -2217},
        {-20057, -2217},
        {-20489, -2196},
        {-20489, -2196},
        {-20893, -2166},
        {-20893, -2166},
        {-21286, -2117},
        {-21286, -2117},
        {-21669, -2052},
        {-21669, -2052},
        {-22048, -1993},
        {-22048, -1993},
        {-22435, -1948},
        {-22435, -1948},
        {-22824, -1931},
        {-22824, -1931},
        {-23212, -1925},
        {-23212, -1925},
        {-23589, -1919},
        {-23589, -1919},
        {-23937, -1889},
        {-23937, -1889},
        {-24267, -1836},
        {-24267, -1836},
        {-24582, -1762},
        {-24582, -1762},
        {-24882, -1674},
        {-24882, -1674},
        {-25161, -1577},
        {-25161, -1577},
        {-25424, -1491},
        {-25424, -1491},
        {-25657, -1392},
        {-25657, -1392},
        {-25857, -1279},
        {-25857, -1279},
        {-26028, -1152},
        {-26028, -1152},
        {-26172, -1016},
        {-26172, -1016},
        {-26293, -876},
        {-26293, -876},
        {-26390, -738},
        {-26390, -738},
        {-26465, -612},
        {-26465, -612},
        {-26541, -517},
        {-26541, -517},
        {-26625, -454},
        {-26625, -454},
        {-26731, -437},
        {-26731, -437},
        {-26834, -445},
        {-26834, -445},
        {-26910, -445},
        {-26910, -445},
        {-26980, -439},
        {-26980, -439},
        {-27003, -382},
        {-27003, -382},
        {-26997, -270},
        {-26997, -270},
        {-26997, -166},
        {-26997, -166},
        {-27003, -96},
        {-27003, -96},
        {-27039, -58},
        {-27039, -58},
        {-27112, -106},
        {-27112, -106},
        {-27174, -176},
        {-27174, -176},
        {-27209, -211},
        {-27209, -211},
        {-27252, -247},
        {-27252, -247},
        {-27291, -254},
        {-27291, -254},
        {-27331, -220},
        {-27331, -220},
        {-27411, -234},
        {-27411, -234},
        {-27503, -292},
        {-27503, -292},
        {-27587, -334},
        {-27587, -334},
        {-27650, -357}
    };

    public static double [][]distalPoints = new double[][]{
        {9752, -21},
        {9664, -106},
        {9664, -106},
        {9664, -106},
        {9664, -106},
        {9492, -275},
        {9492, -275},
        {9492, -275},
        {9492, -275},
        {9258, -507},
        {9258, -507},
        {9258, -507},
        {9258, -507},
        {9009, -756},
        {9009, -756},
        {9009, -756},
        {9009, -756},
        {9009, -756},
        {9009, -756},
        {8741, -1003},
        {8741, -1003},
        {8741, -1003},
        {8741, -1003},
        {8416, -1240},
        {8416, -1240},
        {8006, -1476},
        {8006, -1476},
        {7520, -1729},
        {7520, -1729},
        {6978, -2020},
        {6978, -2020},
        {6373, -2354},
        {6373, -2354},
        {5685, -2715},
        {5685, -2715},
        {4891, -3101},
        {4891, -3101},
        {3987, -3515},
        {3987, -3515},
        {3024, -3937},
        {3024, -3937},
        {2022, -4335},
        {2022, -4335},
        {973, -4698},
        {973, -4698},
        {-85, -4966},
        {-85, -4966},
        {-1132, -5114},
        {-1132, -5114},
        {-2173, -5194},
        {-2173, -5194},
        {-3177, -5202},
        {-3177, -5202},
        {-5130, -5051},
        {-5130, -5051},
        {-5130, -5051},
        {-6107, -4980},
        {-6107, -4980},
        {-7079, -4908},
        {-7079, -4908},
        {-8052, -4874},
        {-8052, -4874},
        {-8994, -4844},
        {-8994, -4844},
        {-9872, -4743},
        {-9872, -4743},
        {-10690, -4591},
        {-10690, -4591},
        {-11455, -4386},
        {-11455, -4386},
        {-12162, -4121},
        {-12162, -4121},
        {-12814, -3832},
        {-12814, -3832},
        {-13440, -3578},
        {-13440, -3578},
        {-14073, -3388},
        {-14073, -3388},
        {-14689, -3241},
        {-14689, -3241},
        {-15263, -3107},
        {-15263, -3107},
        {-15796, -2987},
        {-15796, -2987},
        {-16341, -2904},
        {-16341, -2904},
        {-16896, -2825},
        {-16896, -2825},
        {-17414, -2728},
        {-17414, -2728},
        {-17889, -2630},
        {-17889, -2630},
        {-18335, -2542},
        {-18335, -2542},
        {-18775, -2438},
        {-18775, -2438},
        {-19257, -2362},
        {-19257, -2362},
        {-19776, -2361},
        {-19776, -2361},
        {-20301, -2408},
        {-20301, -2408},
        {-20800, -2463},
        {-20800, -2463},
        {-21282, -2507},
        {-21282, -2507},
        {-21768, -2513},
        {-21768, -2513},
        {-22281, -2504},
        {-22281, -2504},
        {-22824, -2523},
        {-22824, -2523},
        {-23350, -2548},
        {-23350, -2548},
        {-23823, -2542},
        {-23823, -2542},
        {-24244, -2480},
        {-24244, -2480},
        {-24612, -2339},
        {-24612, -2339},
        {-24936, -2121},
        {-24936, -2121},
        {-25220, -1880},
        {-25220, -1880},
        {-25468, -1653},
        {-25468, -1653},
        {-25681, -1445},
        {-25681, -1445},
        {-25864, -1258},
        {-25864, -1258},
        {-26009, -1081},
        {-26009, -1081},
        {-26120, -907},
        {-26120, -907},
        {-26187, -726},
        {-26187, -726},
        {-26228, -553},
        {-26228, -553},
        {-26254, -396},
        {-26254, -396},
        {-26254, -249},
        {-26254, -249},
        {-26252, -136},
        {-26252, -136},
        {-26252, -67},
        {-26252, -67},
        {-26252, -25},
        {-26252, -25},
        {-26252, 1},
        {-26252, 1},
        {-26252, 2},
        {-26252, 2},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0},
        {-26252, 0}
    };

    public static double []wristPoints = new double[]{
        0.550311685,
        0.550311685,
        0.550311685,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551172495,
        0.551172495,
        0.551172495,
        0.551172495,
        0.551172495,
        0.551172495,
        0.551172495,
        0.551172495,
        0.551172495,
        0.551172495,
        0.551172495,
        0.551172495,
        0.551172495,
        0.551172495,
        0.551172495,
        0.551172495,
        0.551172495,
        0.551172495,
        0.551172495,
        0.551172495,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551275373,
        0.551275373,
        0.551275373,
        0.551275373,
        0.551275373,
        0.551275373,
        0.551275373,
        0.551275373,
        0.551275373,
        0.551275373,
        0.551275373,
        0.551275373,
        0.551275373,
        0.551275373,
        0.551275373,
        0.551275373,
        0.551275373,
        0.551275373,
        0.551275373,
        0.551275373,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551276803,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551278353,
        0.551385522,
        0.551385522,
        0.551385522,
        0.551385522,
        0.551385522,
        0.551385522,
        0.551385522,
        0.551385522,
        0.551385522,
        0.551385522,
        0.551385522,
        0.551385522,
        0.551385522,
        0.551385522,
        0.551385522,
        0.551385522,
        0.551385522,
        0.551385522,
        0.551385522,
        0.551385522,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064,
        0.551171064
    };

};
