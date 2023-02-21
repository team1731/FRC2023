package frc.data.mp;

public class PickupLowCone {
    public static ArmPath getArmPath() {
        return new ArmPath(kNumPoints, proximalPoints, distalPoints, kWristFlexIndex, kWristFlexPosition, kWristExtendIndex, kWristMaxVelocity);
    }

    public static final int kNumPoints = 182;
    public static final int kWristFlexIndex = 90;
    public static final double kWristFlexPosition = 0.1874653697013855;
    public static final int kWristExtendIndex = 182;
    public static final double kWristMaxVelocity = 3000;

    // Position (ticks)	Velocity (RPM)

    public static double [][]proximalPoints = new double[][]{
        {-4885.0,  0.0},
        {-4885.0,  0.0},
        {-4885.0,  0.0},
        {-4885.0,  0.0},
        {-4885.0,  0.0},
        {-4885.0,  0.0},
        {-4885.0,  0.0},
        {-4886.0,  0.0},
        {-4886.0,  0.0},
        {-4911.0,  -25.0},
        {-4911.0,  -25.0},
        {-5002.0,  -111.0},
        {-5002.0,  -111.0},
        {-5243.0,  -347.0},
        {-5243.0,  -347.0},
        {-5569.0,  -672.0},
        {-5569.0,  -672.0},
        {-5879.0,  -983.0},
        {-5879.0,  -983.0},
        {-6195.0,  -1272.0},
        {-6195.0,  -1272.0},
        {-6571.0,  -1562.0},
        {-6571.0,  -1562.0},
        {-6974.0,  -1728.0},
        {-6974.0,  -1728.0},
        {-7362.0,  -1789.0},
        {-7362.0,  -1789.0},
        {-7750.0,  -1868.0},
        {-7750.0,  -1868.0},
        {-8150.0,  -1954.0},
        {-8150.0,  -1954.0},
        {-8544.0,  -1972.0},
        {-8544.0,  -1972.0},
        {-8915.0,  -1942.0},
        {-8915.0,  -1942.0},
        {-9290.0,  -1929.0},
        {-9290.0,  -1929.0},
        {-9697.0,  -1947.0},
        {-9697.0,  -1947.0},
        {-10117.0,  -1965.0},
        {-10117.0,  -1965.0},
        {-10548.0,  -2002.0},
        {-10548.0,  -2002.0},
        {-11010.0,  -2091.0},
        {-11010.0,  -2091.0},
        {-11509.0,  -2215.0},
        {-11509.0,  -2215.0},
        {-12042.0,  -2339.0},
        {-12042.0,  -2339.0},
        {-12597.0,  -2473.0},
        {-12597.0,  -2473.0},
        {-13176.0,  -2623.0},
        {-13176.0,  -2623.0},
        {-13778.0,  -2763.0},
        {-13778.0,  -2763.0},
        {-14389.0,  -2876.0},
        {-14389.0,  -2876.0},
        {-15013.0,  -2968.0},
        {-15013.0,  -2968.0},
        {-15638.0,  -3040.0},
        {-15638.0,  -3040.0},
        {-15638.0,  -3040.0},
        {-16908.0,  -3128.0},
        {-16908.0,  -3128.0},
        {-17580.0,  -3189.0},
        {-17580.0,  -3189.0},
        {-18263.0,  -3248.0},
        {-18263.0,  -3248.0},
        {-18901.0,  -3265.0},
        {-18901.0,  -3265.0},
        {-18901.0,  -3265.0},
        {-19513.0,  -3246.0},
        {-19513.0,  -3246.0},
        {-20126.0,  -3219.0},
        {-20126.0,  -3219.0},
        {-20707.0,  -3130.0},
        {-20707.0,  -3130.0},
        {-21251.0,  -2993.0},
        {-21251.0,  -2993.0},
        {-21747.0,  -2850.0},
        {-21747.0,  -2850.0},
        {-21747.0,  -2850.0},
        {-22616.0,  -2498.0},
        {-22616.0,  -2498.0},
        {-22993.0,  -2294.0},
        {-22993.0,  -2294.0},
        {-23329.0,  -2086.0},
        {-23329.0,  -2086.0},
        {-23329.0,  -2086.0},
        {-23625.0,  -1886.0},
        {-23625.0,  -1886.0},
        {-23889.0,  -1691.0},
        {-23889.0,  -1691.0},
        {-24116.0,  -1508.0},
        {-24116.0,  -1508.0},
        {-24306.0,  -1321.0},
        {-24306.0,  -1321.0},
        {-24458.0,  -1136.0},
        {-24458.0,  -1136.0},
        {-24566.0,  -948.0},
        {-24566.0,  -948.0},
        {-24638.0,  -756.0},
        {-24638.0,  -756.0},
        {-24673.0,  -564.0},
        {-24673.0,  -564.0},
        {-24664.0,  -364.0},
        {-24664.0,  -364.0},
        {-24658.0,  -205.0},
        {-24658.0,  -205.0},
        {-24663.0,  -100.0},
        {-24663.0,  -100.0},
        {-24663.0,  -27.0},
        {-24663.0,  -27.0},
        {-24663.0,  10.0},
        {-24663.0,  10.0},
        {-24636.0,  26.0},
        {-24636.0,  26.0},
        {-24536.0,  117.0},
        {-24536.0,  117.0},
        {-24313.0,  340.0},
        {-24313.0,  340.0},
        {-23973.0,  677.0},
        {-23973.0,  677.0},
        {-23618.0,  1031.0},
        {-23618.0,  1031.0},
        {-23293.0,  1333.0},
        {-23293.0,  1333.0},
        {-22968.0,  1561.0},
        {-22968.0,  1561.0},
        {-22624.0,  1686.0},
        {-22624.0,  1686.0},
        {-22284.0,  1690.0},
        {-22284.0,  1690.0},
        {-21959.0,  1661.0},
        {-21959.0,  1661.0},
        {-21656.0,  1638.0},
        {-21656.0,  1638.0},
        {-21383.0,  1587.0},
        {-21383.0,  1587.0},
        {-21133.0,  1495.0},
        {-21133.0,  1495.0},
        {-20903.0,  1385.0},
        {-20903.0,  1385.0},
        {-20903.0,  1385.0},
        {-20512.0,  1148.0},
        {-20512.0,  1148.0},
        {-20359.0,  1028.0},
        {-20359.0,  1028.0},
        {-20236.0,  901.0},
        {-20236.0,  901.0},
        {-20144.0,  764.0},
        {-20144.0,  764.0},
        {-20070.0,  630.0},
        {-20070.0,  630.0},
        {-20049.0,  470.0},
        {-20049.0,  470.0},
        {-20078.0,  289.0},
        {-20078.0,  289.0},
        {-20094.0,  147.0},
        {-20094.0,  147.0},
        {-20081.0,  65.0},
        {-20081.0,  65.0},
        {-20077.0,  -4.0},
        {-20077.0,  -4.0},
        {-20077.0,  -4.0},
        {-20081.0,  -32.0},
        {-20081.0,  -32.0},
        {-20080.0,  -4.0},
        {-20080.0,  -4.0},
        {-20080.0,  14.0},
        {-20080.0,  14.0},
        {-20080.0,  2.0},
        {-20080.0,  2.0},
        {-20080.0,  -3.0},
        {-20080.0,  -3.0},
        {-20080.0,  1.0},
        {-20080.0,  1.0},
        {-20080.0,  0.0},
        {-20080.0,  0.0},
        {-20080.0,  0.0},
        {-20080.0,  0.0},
        {-20080.0,  0.0}
    };

    public static double [][]distalPoints = new double[][]{
        { 11693.0,  0.0},
        { 11693.0,  0.0},
        { 11693.0,  0.0},
        { 11693.0,  0.0},
        { 11693.0,  0.0},
        { 11693.0,  0.0},
        { 11693.0,  0.0},
        { 11693.0,  0.0},
        { 11693.0,  0.0},
        { 11693.0,  0.0},
        { 11693.0,  0.0},
        { 11693.0,  0.0},
        { 11693.0,  0.0},
        { 11693.0,  0.0},
        { 11693.0,  0.0},
        { 11693.0,  0.0},
        { 11693.0,  0.0},
        { 11693.0,  0.0},
        { 11693.0,  0.0},
        { 11700.0,  6.0},
        { 11700.0,  6.0},
        { 11734.0,  39.0},
        { 11734.0,  39.0},
        { 11734.0,  39.0},
        { 11868.0,  172.0},
        { 11868.0,  172.0},
        { 11943.0,  247.0},
        { 11943.0,  247.0},
        { 12048.0,  345.0},
        { 12048.0,  345.0},
        { 12048.0,  345.0},
        { 12183.0,  446.0},
        { 12183.0,  446.0},
        { 12356.0,  561.0},
        { 12356.0,  561.0},
        { 12577.0,  703.0},
        { 12577.0,  703.0},
        { 12853.0,  902.0},
        { 12853.0,  902.0},
        { 13172.0,  1116.0},
        { 13172.0,  1116.0},
        { 13536.0,  1345.0},
        { 13536.0,  1345.0},
        { 13960.0,  1595.0},
        { 13960.0,  1595.0},
        { 14443.0,  1856.0},
        { 14443.0,  1856.0},
        { 14984.0,  2121.0},
        { 14984.0,  2121.0},
        { 15550.0,  2368.0},
        { 15550.0,  2368.0},
        { 16139.0,  2594.0},
        { 16139.0,  2594.0},
        { 16757.0,  2789.0},
        { 16757.0,  2789.0},
        { 17423.0,  2975.0},
        { 17423.0,  2975.0},
        { 18156.0,  3164.0},
        { 18156.0,  3164.0},
        { 18944.0,  3387.0},
        { 18944.0,  3387.0},
        { 18944.0,  3387.0},
        { 20566.0,  3804.0},
        { 20566.0,  3804.0},
        { 21429.0,  3996.0},
        { 21429.0,  3996.0},
        { 22343.0,  4183.0},
        { 22343.0,  4183.0},
        { 23281.0,  4331.0},
        { 23281.0,  4331.0},
        { 23281.0,  4331.0},
        { 24206.0,  4451.0},
        { 24206.0,  4451.0},
        { 25126.0,  4555.0},
        { 25126.0,  4555.0},
        { 25126.0,  4555.0},
        { 26921.0,  4578.0},
        { 26921.0,  4578.0},
        { 27792.0,  4514.0},
        { 27792.0,  4514.0},
        { 28622.0,  4422.0},
        { 28622.0,  4422.0},
        { 28622.0,  4422.0},
        { 29405.0,  4286.0},
        { 29405.0,  4286.0},
        { 30132.0,  4108.0},
        { 30132.0,  4108.0},
        { 30808.0,  3895.0},
        { 30808.0,  3895.0},
        { 31429.0,  3647.0},
        { 31429.0,  3647.0},
        { 31988.0,  3374.0},
        { 31988.0,  3374.0},
        { 32498.0,  3102.0},
        { 32498.0,  3102.0},
        { 32960.0,  2837.0},
        { 32960.0,  2837.0},
        { 33383.0,  2584.0},
        { 33383.0,  2584.0},
        { 33758.0,  2337.0},
        { 33758.0,  2337.0},
        { 34085.0,  2105.0},
        { 34085.0,  2105.0},
        { 34359.0,  1870.0},
        { 34359.0,  1870.0},
        { 34581.0,  1630.0},
        { 34581.0,  1630.0},
        { 34759.0,  1385.0},
        { 34759.0,  1385.0},
        { 34909.0,  1160.0},
        { 34909.0,  1160.0},
        { 35080.0,  1000.0},
        { 35080.0,  1000.0},
        { 35277.0,  920.0},
        { 35277.0,  920.0},
        { 35448.0,  869.0},
        { 35448.0,  869.0},
        { 35588.0,  831.0},
        { 35588.0,  831.0},
        { 35701.0,  793.0},
        { 35701.0,  793.0},
        { 35820.0,  742.0},
        { 35820.0,  742.0},
        { 35978.0,  703.0},
        { 35978.0,  703.0},
        { 36121.0,  674.0},
        { 36121.0,  674.0},
        { 36230.0,  643.0},
        { 36230.0,  643.0},
        { 36310.0,  611.0},
        { 36310.0,  611.0},
        { 36344.0,  528.0},
        { 36344.0,  528.0},
        { 36351.0,  378.0},
        { 36351.0,  378.0},
        { 36350.0,  234.0},
        { 36350.0,  234.0},
        { 36350.0,  124.0},
        { 36350.0,  124.0},
        { 36350.0,  42.0},
        { 36350.0,  42.0},
        { 36350.0,  7.0},
        { 36350.0,  7.0},
        { 36350.0,  7.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0},
        { 36350.0,  0.0}
    };

    public static double []wristPoints = new double[]{
         0.575763463973999,
         0.575763463973999,
         0.575763463973999,
         0.575763463973999,
         0.575763463973999,
         0.575763463973999,
         0.575763463973999,
         0.575763463973999,
         0.575763463973999,
         0.575763463973999,
         0.575763463973999,
         0.575763463973999,
         0.575763463973999,
         0.575763463973999,
         0.575763463973999,
         0.575763463973999,
         0.575763463973999,
         0.575763463973999,
         0.575763463973999,
         0.563973069190979,
         0.563973069190979,
         0.563973069190979,
         0.563973069190979,
         0.563973069190979,
         0.563973069190979,
         0.563973069190979,
         0.563973069190979,
         0.563973069190979,
         0.563973069190979,
         0.563973069190979,
         0.563973069190979,
         0.563973069190979,
         0.563973069190979,
         0.563973069190979,
         0.563973069190979,
         0.563973069190979,
         0.563973069190979,
         0.563973069190979,
         0.563973069190979,
         0.5385693907737732,
         0.5385693907737732,
         0.5385693907737732,
         0.5385693907737732,
         0.5385693907737732,
         0.5385693907737732,
         0.5385693907737732,
         0.5385693907737732,
         0.5385693907737732,
         0.5385693907737732,
         0.5385693907737732,
         0.5385693907737732,
         0.5385693907737732,
         0.5385693907737732,
         0.5385693907737732,
         0.5385693907737732,
         0.5385693907737732,
         0.5385693907737732,
         0.5385693907737732,
         0.5385693907737732,
         0.5044466853141785,
         0.5044466853141785,
         0.5044466853141785,
         0.5044466853141785,
         0.5044466853141785,
         0.5044466853141785,
         0.5044466853141785,
         0.5044466853141785,
         0.5044466853141785,
         0.5044466853141785,
         0.5044466853141785,
         0.5044466853141785,
         0.5044466853141785,
         0.5044466853141785,
         0.5044466853141785,
         0.5044466853141785,
         0.5044466853141785,
         0.5044466853141785,
         0.5044466853141785,
         0.5044466853141785,
         0.437142550945282,
         0.437142550945282,
         0.437142550945282,
         0.437142550945282,
         0.437142550945282,
         0.437142550945282,
         0.437142550945282,
         0.437142550945282,
         0.437142550945282,
         0.437142550945282,
         0.437142550945282,
         0.437142550945282,
         0.437142550945282,
         0.437142550945282,
         0.437142550945282,
         0.437142550945282,
         0.437142550945282,
         0.437142550945282,
         0.437142550945282,
         0.437142550945282,
         0.3356602191925049,
         0.3356602191925049,
         0.3356602191925049,
         0.3356602191925049,
         0.3356602191925049,
         0.3356602191925049,
         0.3356602191925049,
         0.3356602191925049,
         0.3356602191925049,
         0.3356602191925049,
         0.3356602191925049,
         0.3356602191925049,
         0.3356602191925049,
         0.3356602191925049,
         0.3356602191925049,
         0.3356602191925049,
         0.3356602191925049,
         0.3356602191925049,
         0.3356602191925049,
         0.3356602191925049,
         0.26053375005722046,
         0.26053375005722046,
         0.26053375005722046,
         0.26053375005722046,
         0.26053375005722046,
         0.26053375005722046,
         0.26053375005722046,
         0.26053375005722046,
         0.26053375005722046,
         0.26053375005722046,
         0.26053375005722046,
         0.26053375005722046,
         0.26053375005722046,
         0.26053375005722046,
         0.26053375005722046,
         0.26053375005722046,
         0.26053375005722046,
         0.26053375005722046,
         0.26053375005722046,
         0.26053375005722046,
         0.20102542638778687,
         0.20102542638778687,
         0.20102542638778687,
         0.20102542638778687,
         0.20102542638778687,
         0.20102542638778687,
         0.20102542638778687,
         0.20102542638778687,
         0.20102542638778687,
         0.20102542638778687,
         0.20102542638778687,
         0.20102542638778687,
         0.20102542638778687,
         0.20102542638778687,
         0.20102542638778687,
         0.20102542638778687,
         0.20102542638778687,
         0.20102542638778687,
         0.20102542638778687,
         0.20102542638778687,
         0.19131428003311157,
         0.19131428003311157,
         0.19131428003311157,
         0.19131428003311157,
         0.19131428003311157,
         0.19131428003311157,
         0.19131428003311157,
         0.19131428003311157,
         0.19131428003311157,
         0.19131428003311157,
         0.19131428003311157,
         0.19131428003311157,
         0.19131428003311157,
         0.19131428003311157,
         0.19131428003311157,
         0.19131428003311157,
         0.19131428003311157,
         0.19131428003311157,
         0.19131428003311157,
         0.19131428003311157,
         0.1874653697013855,
         0.1874653697013855,
         0.1874653697013855
    };

};