package frc.data.mp;

public class PickupHighConeFeeder {
    public static ArmPath getArmPath() {
        return new ArmPath(kNumPoints, proximalPoints, distalPoints, kWristFlexIndex, kWristFlexPosition, kWristExtendIndex, kWristMaxVelocity);
    }

    public static final int kNumPoints = 76;
    public static final int kWristFlexIndex = 1;
    public static final double kWristFlexPosition = 0.63;
    public static final int kWristExtendIndex = 75;
    public static final double kWristMaxVelocity = 1000;

    // Position (ticks)	Velocity (RPM)

   
    public static double [][]proximalPoints = new double[][]{
        {-4388, -2427},
        {-4776.146667, -2417.666667},
        {-5100.76, -2410.2},
        {-5361.84, -2404.6},
        {-5716.72, -2396.6},
        {-5881, -2394.2},
        {-6045.28, -2391.8},
        {-6209.56, -2389.4},
        {-6469.24, -2387.2},
        {-6635.12, -2387.4},
        {-6987.2, -2380.2},
        {-7339.28, -2373},
        {-7691.36, -2365.8},
        {-7948.04, -2358.4},
        {-8296.32, -2347.4},
        {-8458.4, -2343.8},
        {-8710.88, -2337.4},
        {-8963.36, -2331},
        {-9215.84, -2324.6},
        {-9376.72, -2321.8},
        {-9622.6, -2308.6},
        {-9778.08, -2298.2},
        {-9933.56, -2287.8},
        {-10168.24, -2263.8},
        {-10402.92, -2239.8},
        {-10631, -2211.4},
        {-10859.08, -2183},
        {-11087.16, -2154.6},
        {-11314.04, -2126.6},
        {-11540.92, -2098.6},
        {-11766, -2071.2},
        {-11991.08, -2043.8},
        {-12288.96, -2004.2},
        {-12508.84, -1977.8},
        {-12797.32, -1940.4},
        {-13009.2, -1917.2},
        {-13290.88, -1885.8},
        {-13499.76, -1866.6},
        {-13779.04, -1839.4},
        {-13989.72, -1823.2},
        {-14340, -1797.2},
        {-14620.48, -1779.4},
        {-14900.96, -1756.4},
        {-15174.24, -1741.4},
        {-15447.52, -1726.4},
        {-15638.2, -1708.2},
        {-15828.88, -1690},
        {-16073.16, -1660.8},
        {-16254.24, -1631.6},
        {-16484.12, -1580.8},
        {-16657, -1543},
        {-16872.08, -1478.4},
        {-17033.56, -1430},
        {-17232.04, -1355.4},
        {-17381.72, -1302.4},
        {-17564, -1225},
        {-17704.08, -1174.4},
        {-17872.36, -1098},
        {-18003.64, -1047.8},
        {-18157.92, -972.2},
        {-18279.6, -921},
        {-18420.68, -846.6},
        {-18533.56, -798},
        {-18662.44, -728.4},
        {-18768.32, -684.2},
        {-18888.2, -621.4},
        {-18988.68, -581.8},
        {-19102.36, -527.2},
        {-19200.04, -493.6},
        {-19310.32, -449.2},
        {-19406.6, -423.4},
        {-19512.68, -388.2},
        {-19605.56, -368},
        {-19711.04, -344.2},
        {-19809.52, -323},
        {-19922, -311}
    };

    public static double [][]distalPoints = new double[][]{
        {10300, 2596},
        {10501.13333, 2660.333333},
        {10661.33333, 2711.8},
        {10780.6, 2750.4},
        {11029.86667, 2819},
        {11156.33333, 2849},
        {11282.8, 2879},
        {11409.26667, 2909},
        {11670.53333, 2966.6},
        {11801.8, 2994.2},
        {12210.06667, 3070.8},
        {12618.33333, 3147.4},
        {13026.6, 3224},
        {13300.06667, 3273},
        {13708.13333, 3333.8},
        {13839.2, 3345.6},
        {14101.26667, 3358.6},
        {14363.33333, 3371.6},
        {14625.4, 3384.6},
        {14752.86667, 3385.8},
        {15018.13333, 3389.6},
        {15152.4, 3392.2},
        {15286.66667, 3394.8},
        {15564.53333, 3403.2},
        {15842.4, 3411.6},
        {16122.06667, 3418},
        {16401.73333, 3424.4},
        {16681.4, 3430.8},
        {16950.46667, 3430.4},
        {17219.53333, 3430},
        {17479, 3428},
        {17738.46667, 3426},
        {18255.93333, 3400.8},
        {18640.4, 3376.6},
        {19145.26667, 3333.2},
        {19520.13333, 3290.8},
        {20007, 3227.2},
        {20235.86667, 3186.8},
        {20572.13333, 3123.6},
        {20788, 3079.6},
        {21111.26667, 3013.6},
        {21322.53333, 2968.8},
        {21637.6, 2899.8},
        {21845.26667, 2853.6},
        {22052.93333, 2807.4},
        {22249, 2758.4},
        {22445.06667, 2709.4},
        {22624.73333, 2659.8},
        {22804.4, 2610.2},
        {23063.26667, 2533},
        {23226.33333, 2480.6},
        {23460, 2391.4},
        {23606.26667, 2327},
        {23814.13333, 2220.2},
        {23942.8, 2141},
        {24122.66667, 2017.6},
        {24231.93333, 1931},
        {24382.8, 1798.8},
        {24472.06667, 1709},
        {24595.53333, 1574},
        {24667.8, 1483.2},
        {24767.46667, 1349},
        {24825.53333, 1260.4},
        {24902.6, 1129.2},
        {24945.46667, 1043.2},
        {25001.53333, 919},
        {25030.2, 838.2},
        {25059.26667, 716.2},
        {25069.33333, 636.8},
        {25073, 516.8},
        {25063.46667, 435},
        {25050.53333, 322.2},
        {25037.2, 250.6},
        {25021.86667, 157.8},
        {25010.2, 81.66666667},
        {25000, 11}
    };

    public static double []wristPoints = new double[]{
        0.537484825,
        0.537484825,
        0.537484825,
        0.537484825,
        0.537484825,
        0.537484825,
        0.516007066,
        0.516007066,
        0.516007066,
        0.516007066,
        0.516007066,
        0.516007066,
        0.516007066,
        0.516007066,
        0.516007066,
        0.516007066,
        0.516007066,
        0.516007066,
        0.516007066,
        0.516007066,
        0.516007066,
        0.516007066,
        0.516007066,
        0.516007066,
        0.516007066,
        0.516007066,
        0.516007066,
        0.516007066,
        0.516007066,
        0.516007066,
        0.494566917,
        0.494566917,
        0.494566917,
        0.494566917,
        0.494566917,
        0.494566917,
        0.494566917,
        0.494566917,
        0.494566917,
        0.494566917,
        0.494566917,
        0.494566917,
        0.494566917,
        0.494566917,
        0.494566917,
        0.494566917,
        0.494566917,
        0.494566917,
        0.494566917,
        0.494566917,
        0.470171034,
        0.470171034,
        0.470171034,
        0.470171034,
        0.470171034,
        0.470171034,
        0.470171034,
        0.470171034,
        0.470171034,
        0.470171034,
        0.470171034,
        0.470171034,
        0.470171034,
        0.470171034,
        0.470171034,
        0.470171034,
        0.470171034,
        0.470171034,
        0.470171034,
        0.470171034,
        0.444755197,
        0.444755197,
        0.444755197,
        0.444755197,
        0.444755197,
        0.444755197
    };

};
