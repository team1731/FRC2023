package frc.data.mp;

public class PickupLowThiefInternal {
    public static ArmPath getArmPath() {
        return new ArmPath(kNumPoints, proximalPoints, distalPoints, kWristFlexIndex, kWristFlexPosition, kWristExtendIndex, kWristMaxVelocity);
    }

    public static final int kNumPoints = 46;
    public static final int kWristFlexIndex = 2;
    public static final double kWristFlexPosition = 0.45;
    public static final int kWristExtendIndex = 44;
    public static final double kWristMaxVelocity = 2000;

    // Position (ticks)	Velocity (RPM)

   
  
    public static double [][]proximalPoints = new double[][]{
        {-4388, -31},
        {-4530.422222, -98},
        {-4733.711111, -225.8},
        {-4923.266667, -340.2},
        {-5194.422222, -535.8},
        {-5424.377778, -691.2},
        {-5730.333333, -922.6},
        {-5961.688889, -1079.8},
        {-6268.444444, -1306.6},
        {-6493.6, -1452.2},
        {-6802.955556, -1641.6},
        {-7136.911111, -1781},
        {-7470.866667, -1920.4},
        {-7729.422222, -1990.2},
        {-7987.977778, -2060},
        {-8271.933333, -2113.8},
        {-8455.288889, -2141.6},
        {-8750.644444, -2205.4},
        {-9046, -2269.2},
        {-9451.755556, -2367.8},
        {-9747.911111, -2438.6},
        {-10163.66667, -2544.6},
        {-10467.42222, -2614.6},
        {-10902.17778, -2715.6},
        {-11226.53333, -2781.8},
        {-11685.48889, -2873.2},
        {-12024.84444, -2929.4},
        {-12493.6, -3003.2},
        {-12831.35556, -3046},
        {-13289.91111, -3099.4},
        {-13613.86667, -3127.6},
        {-14053.82222, -3153},
        {-14364.37778, -3160.8},
        {-14791.53333, -3153.4},
        {-15097.88889, -3135.4},
        {-15522.24444, -3101.2},
        {-15830.6, -3069.8},
        {-16257.35556, -3026.8},
        {-16567.51111, -2999},
        {-16987.66667, -2960.6},
        {-17289.82222, -2938.4},
        {-17693.97778, -2902.4},
        {-17979.73333, -2878},
        {-18437.08889, -2791.4},
        {-18829.24444, -2722.333333},
        {-19475, -2515}
    };

    public static double [][]distalPoints = new double[][]{
        {10300, 162},
        {10393.2, 202.3333333},
        {10495.8, 252.4},
        {10590.4, 294.4},
        {10692.2, 343.6},
        {10770, 368.6},
        {10850.2, 387},
        {10913, 387.6},
        {10980.6, 369.6},
        {11041, 344.4},
        {11119.6, 313},
        {11216.8, 291.2},
        {11314, 269.4},
        {11406.4, 266.2},
        {11498.8, 263},
        {11591, 277},
        {11662.2, 288},
        {11756.4, 319},
        {11850.6, 350},
        {11972.8, 404.6},
        {12077, 448.2},
        {12212.4, 504.8},
        {12324.8, 541.4},
        {12470.8, 590.8},
        {12588.8, 616.6},
        {12744.6, 661.6},
        {12869.2, 693.6},
        {13040.8, 749.8},
        {13178.8, 793.2},
        {13375.2, 866.6},
        {13533.8, 920.8},
        {13825.8, 1043.8},
        {14070.8, 1142.6},
        {14384.4, 1272.6},
        {14639.6, 1372.6},
        {14963.6, 1494.4},
        {15154.2, 1547.4},
        {15344.8, 1600.4},
        {15535.6, 1632.8},
        {15726.4, 1665.2},
        {15917, 1679.2},
        {16107.6, 1693.2},
        {16365.8, 1706.6},
        {16555.2, 1709.4},
        {16767.13333, 1712},
        {16933, 1711}
    };

    public static double []wristPoints = new double[]{
        0.506325424,
        0.506325424,
        0.506325424,
        0.506325424,
        0.506325424,
        0.506325424,
        0.506325424,
        0.506325424,
        0.506325424,
        0.506325424,
        0.506325424,
        0.506325424,
        0.506325424,
        0.460502505,
        0.460502505,
        0.460502505,
        0.460502505,
        0.460502505,
        0.460502505,
        0.460502505,
        0.460502505,
        0.460502505,
        0.460502505,
        0.460502505,
        0.460502505,
        0.460502505,
        0.460502505,
        0.460502505,
        0.460502505,
        0.460502505,
        0.460502505,
        0.460502505,
        0.460502505,
        0.460502505,
        0.379562914,
        0.379562914,
        0.379562914,
        0.379562914,
        0.379562914,
        0.379562914,
        0.379562914,
        0.379562914,
        0.379562914,
        0.379562914,
        0.379562914,
        0.379562914
    };

};
