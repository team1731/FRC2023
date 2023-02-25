package frc.data.mp;

public class ScoreHighCube {
    public static ArmPath getArmPath() {
        return new ArmPath(kNumPoints, proximalPoints, distalPoints, kWristFlexIndex, kWristFlexPosition, kWristExtendIndex, kWristMaxVelocity);
    }

    public static final int kNumPoints = 225;
    public static final int kWristFlexIndex = 75;
    public static final double kWristFlexPosition = 0.419;
    public static final int kWristExtendIndex = 220;
    public static final double kWristMaxVelocity = 2000;

    // Position (ticks)	Velocity (RPM)

    public static double [][]proximalPoints = new double[][]{
        {-3921, -1},
        {-3921, -1},
        {-3922, -2},
        {-3922, -2},
        {-3924, -4},
        {-3924, -4},
        {-3925, -5},
        {-3925, -5},
        {-3926, -6},
        {-3926, -6},
        {-3926, -5},
        {-3926, -5},
        {-3926, -4},
        {-3926, -4},
        {-3926, -2},
        {-3926, -2},
        {-3926, -1},
        {-3926, -1},
        {-3928, -2},
        {-3928, -2},
        {-3939, -12},
        {-3939, -12},
        {-3982, -55},
        {-3982, -55},
        {-4076, -149},
        {-4076, -149},
        {-4190, -263},
        {-4190, -263},
        {-4312, -383},
        {-4312, -383},
        {-4442, -502},
        {-4442, -502},
        {-4584, -601},
        {-4584, -601},
        {-4752, -674},
        {-4752, -674},
        {-4954, -763},
        {-4954, -763},
        {-5193, -880},
        {-5193, -880},
        {-5463, -1020},
        {-5463, -1020},
        {-5783, -1198},
        {-5783, -1198},
        {-6149, -1396},
        {-6149, -1396},
        {-6544, -1587},
        {-6544, -1587},
        {-6966, -1769},
        {-6966, -1769},
        {-7405, -1941},
        {-7405, -1941},
        {-7858, -2074},
        {-7858, -2074},
        {-8327, -2177},
        {-8327, -2177},
        {-8804, -2260},
        {-8804, -2260},
        {-9304, -2338},
        {-9304, -2338},
        {-9813, -2405},
        {-9813, -2405},
        {-10352, -2493},
        {-10352, -2493},
        {-10915, -2588},
        {-10915, -2588},
        {-11507, -2702},
        {-11507, -2702},
        {-12108, -2803},
        {-12108, -2803},
        {-12718, -2906},
        {-12718, -2906},
        {-13353, -2998},
        {-13353, -2998},
        {-13983, -3067},
        {-13983, -3067},
        {-14614, -3106},
        {-14614, -3106},
        {-15244, -3137},
        {-15244, -3137},
        {-15902, -3182},
        {-15902, -3182},
        {-16607, -3254},
        {-16607, -3254},
        {-17332, -3349},
        {-17332, -3349},
        {-18058, -3442},
        {-18058, -3442},
        {-18781, -3535},
        {-18781, -3535},
        {-19494, -3592},
        {-19494, -3592},
        {-20175, -3570},
        {-20175, -3570},
        {-20838, -3505},
        {-20838, -3505},
        {-21504, -3448},
        {-21504, -3448},
        {-22163, -3383},
        {-22163, -3383},
        {-22825, -3332},
        {-22825, -3332},
        {-23475, -3300},
        {-23475, -3300},
        {-24109, -3271},
        {-24109, -3271},
        {-24729, -3226},
        {-24729, -3226},
        {-25343, -3179},
        {-25343, -3179},
        {-25949, -3124},
        {-25949, -3124},
        {-26534, -3060},
        {-26534, -3060},
        {-27093, -2986},
        {-27093, -2986},
        {-27643, -2916},
        {-27643, -2916},
        {-28186, -2846},
        {-28186, -2846},
        {-28744, -2796},
        {-28744, -2796},
        {-29284, -2750},
        {-29284, -2750},
        {-29824, -2730},
        {-29824, -2730},
        {-30355, -2711},
        {-30355, -2711},
        {-30904, -2718},
        {-30904, -2718},
        {-31463, -2720},
        {-31463, -2720},
        {-32033, -2749},
        {-32033, -2749},
        {-32601, -2777},
        {-32601, -2777},
        {-33168, -2813},
        {-33168, -2813},
        {-33783, -2878},
        {-33783, -2878},
        {-34440, -2973},
        {-34440, -2973},
        {-35117, -3082},
        {-35117, -3082},
        {-35782, -3181},
        {-35782, -3181},
        {-36412, -3243},
        {-36412, -3243},
        {-37015, -3231},
        {-37015, -3231},
        {-37587, -3150},
        {-37587, -3150},
        {-38148, -3031},
        {-38148, -3031},
        {-38698, -2916},
        {-38698, -2916},
        {-39269, -2857},
        {-39269, -2857},
        {-39269, -2857},
        {-39862, -2848},
        {-39862, -2848},
        {-40454, -2866},
        {-40454, -2866},
        {-41033, -2887},
        {-41033, -2887},
        {-41570, -2873},
        {-41570, -2873},
        {-42083, -2816},
        {-42083, -2816},
        {-42570, -2709},
        {-42570, -2709},
        {-43043, -2591},
        {-43043, -2591},
        {-43043, -2591},
        {-43879, -2311},
        {-43879, -2311},
        {-44241, -2159},
        {-44568, -1999},
        {-44568, -1999},
        {-44568, -1999},
        {-44568, -1999},
        {-44890, -1849},
        {-44890, -1849},
        {-45221, -1744},
        {-45221, -1744},
        {-45590, -1710},
        {-45590, -1710},
        {-46000, -1758},
        {-46000, -1758},
        {-46401, -1833},
        {-46401, -1833},
        {-46776, -1885},
        {-46776, -1885},
        {-47123, -1904},
        {-47123, -1904},
        {-47437, -1848},
        {-47437, -1848},
        {-47718, -1719},
        {-47718, -1719},
        {-47971, -1572},
        {-47971, -1572},
        {-48193, -1418},
        {-48193, -1418},
        {-48386, -1263},
        {-48386, -1263},
        {-48550, -1114},
        {-48550, -1114},
        {-48690, -974},
        {-48690, -974},
        {-48810, -840},
        {-48810, -840},
        {-48929, -544},
        {-48929, -544},
        {-48929, -544},
        {-48973, -425},
        {-48972, -283},
        {-48972, -283},
        {-48972, -283},
        {-48972, -163},
        {-48972, -163},
        {-48972, -93},
        {-48972, -93},
        {-48972, -44},
        {-48972, -44},
        {-48972, 1}
    };

    public static double [][]distalPoints = new double[][]{
        {-3649, -4073},
        {-3649, -4073},
        {-4545, -4203},
        {-4545, -4203},
        {-5502, -4406},
        {-5502, -4406},
        {-6515, -4625},
        {-6515, -4625},
        {-7560, -4804},
        {-7560, -4804},
        {-8624, -4972},
        {-8624, -4972},
        {-9700, -5153},
        {-9700, -5153},
        {-10755, -5251},
        {-10755, -5251},
        {-11832, -5317},
        {-11832, -5317},
        {-12905, -5345},
        {-12905, -5345},
        {-13967, -5344},
        {-13967, -5344},
        {-15042, -5344},
        {-15042, -5344},
        {-16113, -5359},
        {-16113, -5359},
        {-17159, -5326},
        {-17159, -5326},
        {-18217, -5313},
        {-18217, -5313},
        {-19250, -5285},
        {-19250, -5285},
        {-20272, -5228},
        {-20272, -5228},
        {-21314, -5201},
        {-21314, -5201},
        {-22347, -5189},
        {-22347, -5189},
        {-23371, -5154},
        {-23371, -5154},
        {-24435, -5183},
        {-24435, -5183},
        {-25501, -5232},
        {-25501, -5232},
        {-26587, -5272},
        {-26587, -5272},
        {-27685, -5336},
        {-27685, -5336},
        {-28738, -5367},
        {-28738, -5367},
        {-29783, -5348},
        {-29783, -5348},
        {-30826, -5323},
        {-30826, -5323},
        {-31844, -5259},
        {-31844, -5259},
        {-32849, -5165},
        {-32849, -5165},
        {-33836, -5099},
        {-33836, -5099},
        {-34776, -4993},
        {-34776, -4993},
        {-35678, -4853},
        {-35678, -4853},
        {-36593, -4749},
        {-36593, -4749},
        {-37485, -4639},
        {-37485, -4639},
        {-38335, -4497},
        {-38335, -4497},
        {-39140, -4367},
        {-39140, -4367},
        {-39932, -4255},
        {-39932, -4255},
        {-40688, -4097},
        {-40688, -4097},
        {-41416, -3932},
        {-41416, -3932},
        {-42163, -3832},
        {-42163, -3832},
        {-42909, -3768},
        {-42909, -3768},
        {-43615, -3683},
        {-43615, -3683},
        {-44273, -3587},
        {-44273, -3587},
        {-44914, -3498},
        {-44914, -3498},
        {-45567, -3403},
        {-45567, -3403},
        {-46220, -3312},
        {-46220, -3312},
        {-46844, -3231},
        {-46844, -3231},
        {-47443, -3170},
        {-47443, -3170},
        {-48056, -3143},
        {-48056, -3143},
        {-48659, -3092},
        {-48659, -3092},
        {-49210, -2992},
        {-49210, -2992},
        {-49714, -2870},
        {-49714, -2870},
        {-50168, -2724},
        {-50168, -2724},
        {-50579, -2524},
        {-50579, -2524},
        {-50949, -2293},
        {-50949, -2293},
        {-51278, -2070},
        {-51278, -2070},
        {-51569, -1859},
        {-51569, -1859},
        {-51818, -1654},
        {-51818, -1654},
        {-52019, -1443},
        {-52019, -1443},
        {-52180, -1234},
        {-52180, -1234},
        {-52303, -1027},
        {-52303, -1027},
        {-52388, -820},
        {-52388, -820},
        {-52447, -631},
        {-52447, -631},
        {-52464, -446},
        {-52464, -446},
        {-52440, -261},
        {-52440, -261},
        {-52428, -126},
        {-52428, -126},
        {-52429, -43},
        {-52429, -43},
        {-52399, 48},
        {-52399, 48},
        {-52316, 146},
        {-52316, 146},
        {-52222, 218},
        {-52222, 218},
        {-52147, 281},
        {-52147, 281},
        {-52126, 304},
        {-52126, 304},
        {-52145, 254},
        {-52145, 254},
        {-52177, 141},
        {-52177, 141},
        {-52172, 50},
        {-52172, 50},
        {-52158, -11},
        {-52158, -11},
        {-52163, -37},
        {-52163, -37},
        {-52162, -17},
        {-52162, -17},
        {-52162, 15},
        {-52162, 15},
        {-52162, 15},
        {-52162, 10},
        {-52162, 10},
        {-52162, -4},
        {-52162, -4},
        {-52162, 1},
        {-52162, 1},
        {-52162, 0},
        {-52162, 0},
        {-52162, 0},
        {-52162, 0},
        {-52162, 0},
        {-52162, 0},
        {-52162, 0},
        {-52162, 0},
        {-52162, 0},
        {-52162, 0},
        {-52162, 0},
        {-52162, 0},
        {-52161, 1},
        {-52161, 1},
        {-52161, 1},
        {-52161, 1},
        {-52151, 11},
        {-52151, 11},
        {-52133, 29},
        {-52133, 29},
        {-52095, 66},
        {-52095, 66},
        {-52056, 106},
        {-52056, 106},
        {-52021, 139},
        {-52021, 139},
        {-52006, 145},
        {-52006, 145},
        {-52011, 122},
        {-52011, 122},
        {-52011, 85},
        {-52011, 85},
        {-52011, 45},
        {-52011, 45},
        {-52011, 11},
        {-52011, 11},
        {-52011, -5},
        {-52011, -5},
        {-52011, 0},
        {-52011, 0},
        {-52011, 0},
        {-52011, 0},
        {-52011, 0},
        {-52011, 0},
        {-52011, 0},
        {-52011, 0},
        {-52011, 0},
        {-52011, 0},
        {-52011, 0},
        {-52011, 0},
        {-52011, 0},
        {-52011, 0},
        {-52011, 0},
        {-52011, 0},
        {-52011, 0},
        {-52011, 0},
        {-52011, 0},
        {-52011, 0},
        {-52011, 0},
        {-52011, 0}
    };

    public static double []wristPoints = new double[]{
        0.499594837,
        0.499594837,
        0.499594837,
        0.499594837,
        0.499601811,
        0.499601811,
        0.499601811,
        0.499601811,
        0.499601811,
        0.499601811,
        0.499601811,
        0.499601811,
        0.499601811,
        0.499601811,
        0.499601811,
        0.499601811,
        0.499601811,
        0.499601811,
        0.499601811,
        0.499601811,
        0.499601811,
        0.499601811,
        0.499601811,
        0.499601811,
        0.499587804,
        0.499587804,
        0.499587804,
        0.499587804,
        0.499587804,
        0.499587804,
        0.499587804,
        0.499587804,
        0.499587804,
        0.499587804,
        0.499587804,
        0.499587804,
        0.499587804,
        0.499587804,
        0.499587804,
        0.499587804,
        0.499587804,
        0.499587804,
        0.499587804,
        0.499587804,
        0.499587804,
        0.498636872,
        0.498636872,
        0.498636872,
        0.498636872,
        0.498636872,
        0.498636872,
        0.498636872,
        0.498636872,
        0.498636872,
        0.498636872,
        0.498636872,
        0.498636872,
        0.498636872,
        0.498636872,
        0.498636872,
        0.498636872,
        0.498636872,
        0.498636872,
        0.498636872,
        0.488773078,
        0.488773078,
        0.488773078,
        0.488773078,
        0.488773078,
        0.488773078,
        0.488773078,
        0.488773078,
        0.488773078,
        0.488773078,
        0.488773078,
        0.488773078,
        0.488773078,
        0.488773078,
        0.488773078,
        0.488773078,
        0.488773078,
        0.488773078,
        0.488773078,
        0.488773078,
        0.473216981,
        0.473216981,
        0.473216981,
        0.473216981,
        0.473216981,
        0.473216981,
        0.473216981,
        0.473216981,
        0.473216981,
        0.473216981,
        0.473216981,
        0.473216981,
        0.473216981,
        0.473216981,
        0.473216981,
        0.473216981,
        0.473216981,
        0.473216981,
        0.473216981,
        0.473216981,
        0.438046485,
        0.438046485,
        0.438046485,
        0.438046485,
        0.438046485,
        0.438046485,
        0.438046485,
        0.438046485,
        0.438046485,
        0.438046485,
        0.438046485,
        0.438046485,
        0.438046485,
        0.438046485,
        0.438046485,
        0.438046485,
        0.438046485,
        0.438046485,
        0.438046485,
        0.438046485,
        0.438046485,
        0.41072008,
        0.41072008,
        0.41072008,
        0.41072008,
        0.41072008,
        0.41072008,
        0.41072008,
        0.41072008,
        0.41072008,
        0.41072008,
        0.41072008,
        0.41072008,
        0.41072008,
        0.41072008,
        0.41072008,
        0.41072008,
        0.41072008,
        0.41072008,
        0.41072008,
        0.413631111,
        0.413631111,
        0.413631111,
        0.413631111,
        0.413631111,
        0.413631111,
        0.413631111,
        0.413631111,
        0.413631111,
        0.413631111,
        0.413631111,
        0.413631111,
        0.413631111,
        0.413631111,
        0.413631111,
        0.413631111,
        0.413631111,
        0.413631111,
        0.413631111,
        0.413631111,
        0.418546826,
        0.418546826,
        0.418546826,
        0.418546826,
        0.418546826,
        0.418546826,
        0.418546826,
        0.418546826,
        0.418546826,
        0.418546826,
        0.418546826,
        0.418546826,
        0.418546826,
        0.418546826,
        0.418546826,
        0.418546826,
        0.418546826,
        0.418546826,
        0.418546826,
        0.418546826,
        0.421441644,
        0.421441644,
        0.421441644,
        0.421441644,
        0.421441644,
        0.421441644,
        0.421441644,
        0.421441644,
        0.421441644,
        0.421441644,
        0.421441644,
        0.421441644,
        0.421441644,
        0.421441644,
        0.421441644,
        0.421441644,
        0.421441644,
        0.421441644,
        0.421441644,
        0.421441644,
        0.419511765,
        0.419511765,
        0.419511765,
        0.419511765,
        0.419511765,
        0.419511765,
        0.419511765,
        0.419511765,
        0.419511765,
        0.419511765,
        0.419511765,
        0.419511765,
        0.419511765,
        0.419511765,
        0.419511765,
        0.419511765,
        0.419511765,
        0.419511765,
        0.419511765,
        0.419511765,
        0.419587821
    };

};
