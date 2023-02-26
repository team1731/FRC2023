package frc.data.mp;

public class ScoreMediumCone {
    public static ArmPath getArmPath() {
        return new ArmPath(kNumPoints, proximalPoints, distalPoints, kWristFlexIndex, kWristFlexPosition, kWristExtendIndex, kWristMaxVelocity);
    }

    public static final int kNumPoints = 190;
    public static final int kWristFlexIndex = 75;
    public static final double kWristFlexPosition = 0.408;
    public static final int kWristExtendIndex = 185;
    public static final double kWristMaxVelocity = 2000;

    // Position (ticks)	Velocity (RPM)

    public static double [][]proximalPoints = new double[][]{
        {-3936, -647},
        {-3936, -647},
        {-3936, -647},
        {-4031, -582},
        {-4031, -582},
        {-4111, -510},
        {-4111, -510},
        {-4179, -445},
        {-4179, -445},
        {-4179, -445},
        {-4208, -366},
        {-4208, -366},
        {-4228, -197},
        {-4228, -197},
        {-4228, -197},
        {-4227, -116},
        {-4227, -116},
        {-4227, -49},
        {-4227, -49},
        {-4228, -20},
        {-4228, -20},
        {-4228, 2},
        {-4228, 2},
        {-4236, -8},
        {-4236, -8},
        {-4274, -47},
        {-4274, -47},
        {-4368, -141},
        {-4368, -141},
        {-4521, -292},
        {-4521, -292},
        {-4690, -461},
        {-4690, -461},
        {-4861, -624},
        {-4861, -624},
        {-5026, -751},
        {-5026, -751},
        {-5181, -813},
        {-5181, -813},
        {-5327, -806},
        {-5327, -806},
        {-5327, -806},
        {-5463, -773},
        {-5463, -773},
        {-5608, -747},
        {-5608, -747},
        {-5775, -749},
        {-5775, -749},
        {-5960, -778},
        {-5960, -778},
        {-6146, -819},
        {-6146, -819},
        {-6329, -865},
        {-6329, -865},
        {-6523, -915},
        {-6523, -915},
        {-6743, -967},
        {-6743, -967},
        {-6983, -1023},
        {-6983, -1023},
        {-7247, -1100},
        {-7247, -1100},
        {-7519, -1191},
        {-7519, -1191},
        {-7791, -1268},
        {-7791, -1268},
        {-8076, -1333},
        {-8076, -1333},
        {-8367, -1383},
        {-8367, -1383},
        {-8667, -1420},
        {-8667, -1420},
        {-8986, -1465},
        {-8986, -1465},
        {-9333, -1540},
        {-9333, -1540},
        {-9693, -1617},
        {-9693, -1617},
        {-10054, -1687},
        {-10054, -1687},
        {-10424, -1756},
        {-10424, -1756},
        {-10781, -1796},
        {-10781, -1796},
        {-11528, -1835},
        {-11528, -1835},
        {-11528, -1835},
        {-11918, -1864},
        {-11918, -1864},
        {-12313, -1890},
        {-12313, -1890},
        {-12697, -1916},
        {-12697, -1916},
        {-13065, -1920},
        {-13065, -1920},
        {-13424, -1896},
        {-13424, -1896},
        {-13778, -1860},
        {-13778, -1860},
        {-14139, -1826},
        {-14139, -1826},
        {-14525, -1828},
        {-14525, -1828},
        {-14919, -1855},
        {-14919, -1855},
        {-15307, -1883},
        {-15307, -1883},
        {-15683, -1905},
        {-15683, -1905},
        {-16042, -1903},
        {-16042, -1903},
        {-16406, -1881},
        {-16406, -1881},
        {-16767, -1847},
        {-16767, -1847},
        {-17118, -1811},
        {-17118, -1811},
        {-17472, -1789},
        {-17472, -1789},
        {-17823, -1781},
        {-17823, -1781},
        {-18166, -1760},
        {-18166, -1760},
        {-18510, -1744},
        {-18510, -1744},
        {-18847, -1729},
        {-18847, -1729},
        {-19174, -1703},
        {-19174, -1703},
        {-19496, -1674},
        {-19496, -1674},
        {-19807, -1641},
        {-19807, -1641},
        {-20109, -1599},
        {-20109, -1599},
        {-20409, -1562},
        {-20409, -1562},
        {-20705, -1530},
        {-20705, -1530},
        {-21004, -1508},
        {-21004, -1508},
        {-21314, -1507},
        {-21314, -1507},
        {-21623, -1514},
        {-21623, -1514},
        {-21899, -1490},
        {-21899, -1490},
        {-22134, -1429},
        {-22134, -1429},
        {-22330, -1326},
        {-22330, -1326},
        {-22494, -1181},
        {-22494, -1181},
        {-22625, -1003},
        {-22625, -1003},
        {-22731, -833},
        {-22731, -833},
        {-22813, -681},
        {-22813, -681},
        {-22900, -570},
        {-22900, -570},
        {-22980, -486},
        {-22980, -486},
        {-23052, -428},
        {-23052, -428},
        {-23086, -356},
        {-23086, -356},
        {-23134, -320},
        {-23134, -320},
        {-23168, -268},
        {-23168, -268},
        {-23158, -179},
        {-23158, -179},
        {-23159, -106},
        {-23159, -106},
        {-23159, -73},
        {-23159, -73},
        {-23159, -26},
        {-23159, -26},
        {-23159, 8},
        {-23159, 8},
        {-23162, -4},
        {-23162, -4},
        {-23166, -8},
        {-23166, -8},
        {-23168, -9},
        {-23168, -9},
        {-23170, -11},
        {-23170, -11},
        {-23170, -11}
    };

    public static double [][]distalPoints = new double[][]{
        {5081, -2516},
        {5081, -2516},
        {5081, -2516},
        {4438, -2722},
        {4438, -2722},
        {3708, -2974},
        {3708, -2974},
        {2015, -3646},
        {2015, -3646},
        {2015, -3646},
        {1071, -4010},
        {1071, -4010},
        {49, -4387},
        {49, -4387},
        {49, -4387},
        {-994, -4700},
        {-994, -4700},
        {-2013, -4908},
        {-2013, -4908},
        {-3019, -5033},
        {-3019, -5033},
        {-4003, -5075},
        {-4003, -5075},
        {-4943, -4990},
        {-4943, -4990},
        {-5881, -4888},
        {-5881, -4888},
        {-6836, -4823},
        {-6836, -4823},
        {-7820, -4800},
        {-7820, -4800},
        {-8829, -4823},
        {-8829, -4823},
        {-9855, -4913},
        {-9855, -4913},
        {-10854, -4974},
        {-10854, -4974},
        {-11870, -5034},
        {-11870, -5034},
        {-12891, -5073},
        {-12891, -5073},
        {-12891, -5073},
        {-14916, -5060},
        {-14916, -5060},
        {-15947, -5090},
        {-15947, -5090},
        {-16971, -5102},
        {-16971, -5102},
        {-18018, -5126},
        {-18018, -5126},
        {-19063, -5159},
        {-19063, -5159},
        {-20089, -5175},
        {-20089, -5175},
        {-21122, -5175},
        {-21122, -5175},
        {-22167, -5195},
        {-22167, -5195},
        {-23163, -5145},
        {-23163, -5145},
        {-24145, -5084},
        {-24145, -5084},
        {-25120, -5031},
        {-25120, -5031},
        {-26066, -4945},
        {-26066, -4945},
        {-26996, -4829},
        {-26996, -4829},
        {-27912, -4750},
        {-27912, -4750},
        {-28781, -4634},
        {-28781, -4634},
        {-29599, -4480},
        {-29599, -4480},
        {-30399, -4333},
        {-30399, -4333},
        {-31168, -4172},
        {-31168, -4172},
        {-31906, -3995},
        {-31906, -3995},
        {-31906, -3995},
        {-32625, -3846},
        {-32625, -3846},
        {-33332, -3734},
        {-33332, -3734},
        {-34031, -3635},
        {-34031, -3635},
        {-34692, -3527},
        {-34692, -3527},
        {-35302, -3397},
        {-35302, -3397},
        {-35878, -3254},
        {-35878, -3254},
        {-36453, -3121},
        {-36453, -3121},
        {-37011, -2980},
        {-37011, -2980},
        {-37524, -2832},
        {-37524, -2832},
        {-37997, -2696},
        {-37997, -2696},
        {-38430, -2553},
        {-38430, -2553},
        {-38836, -2383},
        {-38836, -2383},
        {-39240, -2229},
        {-39240, -2229},
        {-39647, -2125},
        {-39647, -2125},
        {-40029, -2033},
        {-40029, -2033},
        {-40373, -1943},
        {-40373, -1943},
        {-40678, -1844},
        {-40678, -1844},
        {-40947, -1709},
        {-40947, -1709},
        {-41177, -1529},
        {-41177, -1529},
        {-41368, -1339},
        {-41368, -1339},
        {-41521, -1149},
        {-41521, -1149},
        {-41633, -955},
        {-41633, -955},
        {-41703, -756},
        {-41703, -756},
        {-41733, -558},
        {-41733, -558},
        {-41739, -372},
        {-41739, -372},
        {-41736, -216},
        {-41736, -216},
        {-41735, -103},
        {-41735, -103},
        {-41734, -32},
        {-41734, -32},
        {-41696, 37},
        {-41696, 37},
        {-41537, 201},
        {-41537, 201},
        {-41292, 442},
        {-41292, 442},
        {-41035, 699},
        {-41035, 699},
        {-40801, 932},
        {-40801, 932},
        {-40583, 1112},
        {-40583, 1112},
        {-40366, 1171},
        {-40366, 1171},
        {-40121, 1171},
        {-40121, 1171},
        {-39863, 1172},
        {-39863, 1172},
        {-39615, 1186},
        {-39615, 1186},
        {-39391, 1191},
        {-39391, 1191},
        {-39197, 1169},
        {-39197, 1169},
        {-39023, 1099},
        {-39023, 1099},
        {-38872, 991},
        {-38872, 991},
        {-38749, 866},
        {-38749, 866},
        {-38654, 738},
        {-38654, 738},
        {-38592, 605},
        {-38592, 605},
        {-38548, 476},
        {-38548, 476},
        {-38542, 331},
        {-38542, 331},
        {-38559, 191},
        {-38559, 191},
        {-38573, 82},
        {-38573, 82},
        {-38585, 8},
        {-38585, 8},
        {-38584, -36},
        {-38584, -36},
        {-38584, -42},
        {-38584, -42},
        {-38584, -25},
        {-38584, -25},
        {-38584, -11},
        {-38584, -11},
        {-38584, 1}
    };

    public static double []wristPoints = new double[]{
        0.442952782,
        0.442952782,
        0.442952782,
        0.442952782,
        0.442952782,
        0.442952782,
        0.442952782,
        0.442952782,
        0.442952782,
        0.442952782,
        0.442952782,
        0.442952782,
        0.442952782,
        0.442952782,
        0.442952782,
        0.414547592,
        0.414547592,
        0.414547592,
        0.414547592,
        0.414547592,
        0.414547592,
        0.414547592,
        0.414547592,
        0.414547592,
        0.414547592,
        0.414547592,
        0.414547592,
        0.414547592,
        0.414547592,
        0.414547592,
        0.414547592,
        0.414547592,
        0.414547592,
        0.414547592,
        0.414547592,
        0.405934423,
        0.405934423,
        0.405934423,
        0.405934423,
        0.405934423,
        0.405934423,
        0.405934423,
        0.405934423,
        0.405934423,
        0.405934423,
        0.405934423,
        0.405934423,
        0.405934423,
        0.405934423,
        0.405934423,
        0.405934423,
        0.405934423,
        0.405934423,
        0.405934423,
        0.405934423,
        0.423433274,
        0.423433274,
        0.423433274,
        0.423433274,
        0.423433274,
        0.423433274,
        0.423433274,
        0.423433274,
        0.423433274,
        0.423433274,
        0.423433274,
        0.423433274,
        0.423433274,
        0.423433274,
        0.423433274,
        0.423433274,
        0.423433274,
        0.423433274,
        0.423433274,
        0.423433274,
        0.426314145,
        0.426314145,
        0.426314145,
        0.426314145,
        0.426314145,
        0.426314145,
        0.426314145,
        0.426314145,
        0.426314145,
        0.426314145,
        0.426314145,
        0.426314145,
        0.426314145,
        0.426314145,
        0.426314145,
        0.426314145,
        0.426314145,
        0.426314145,
        0.426314145,
        0.426314145,
        0.424291223,
        0.424291223,
        0.424291223,
        0.424291223,
        0.424291223,
        0.424291223,
        0.424291223,
        0.424291223,
        0.424291223,
        0.424291223,
        0.424291223,
        0.424291223,
        0.424291223,
        0.424291223,
        0.424291223,
        0.424291223,
        0.424291223,
        0.424291223,
        0.424291223,
        0.424291223,
        0.421502739,
        0.421502739,
        0.421502739,
        0.421502739,
        0.421502739,
        0.421502739,
        0.421502739,
        0.421502739,
        0.421502739,
        0.421502739,
        0.421502739,
        0.421502739,
        0.421502739,
        0.421502739,
        0.421502739,
        0.421502739,
        0.421502739,
        0.421502739,
        0.421502739,
        0.421502739,
        0.412708253,
        0.412708253,
        0.412708253,
        0.412708253,
        0.412708253,
        0.412708253,
        0.412708253,
        0.412708253,
        0.412708253,
        0.412708253,
        0.412708253,
        0.412708253,
        0.412708253,
        0.412708253,
        0.412708253,
        0.412708253,
        0.412708253,
        0.412708253,
        0.412708253,
        0.412708253,
        0.411726564,
        0.411726564,
        0.411726564,
        0.411726564,
        0.411726564,
        0.411726564,
        0.411726564,
        0.411726564,
        0.411726564,
        0.411726564,
        0.411726564,
        0.411726564,
        0.411726564,
        0.411726564,
        0.411726564,
        0.411726564,
        0.411726564,
        0.411726564,
        0.411726564,
        0.411726564,
        0.408847243,
        0.408847243,
        0.408847243,
        0.408847243,
        0.408847243,
        0.408847243,
        0.408847243,
        0.408847243,
        0.408847243,
        0.408847243,
        0.408847243,
        0.408847243,
        0.408847243,
        0.408847243,
        0.408847243
    };

};