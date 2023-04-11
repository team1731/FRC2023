package frc.data.mp;

public class ScoreHighConeAuto {
    public static ArmPath getArmPath() {
        return new ArmPath(kNumPoints, proximalPoints, distalPoints, kWristFlexIndex, kWristFlexPosition, kWristExtendIndex, kWristMaxVelocity);
    }

    public static final int kNumPoints = 100;
    public static final int kWristFlexIndex = 50;
    public static final double kWristFlexPosition = 0.35;
    public static final int kWristExtendIndex = 90;
    public static final double kWristMaxVelocity = 2000;

    public static double [][]proximalPoints = new double[][]{
        {-4388, -104},
        {-4498.581818, -104},
        {-4660.563636, -154.8},
        {-4822.545455, -205.6},
        {-5049.927273, -321.8},
        {-5277.309091, -438},
        {-5566.690909, -616.4},
        {-5804.672727, -744},
        {-6096.854545, -925.2},
        {-6323.636364, -1041},
        {-6601.618182, -1187.8},
        {-6817.6, -1272.4},
        {-7087.581818, -1360},
        {-7303.363636, -1394},
        {-7568.945455, -1412.6},
        {-7783.327273, -1400.2},
        {-8036.509091, -1364.8},
        {-8235.690909, -1326.4},
        {-8462.872727, -1262},
        {-8640.254545, -1213},
        {-8836.236364, -1131.6},
        {-8993.418182, -1073.2},
        {-9163, -974},
        {-9304.581818, -900.8},
        {-9445.563636, -777.2},
        {-9567.945455, -686},
        {-9684.927273, -550.2},
        {-9789.509091, -455.2},
        {-9890.290909, -327.8},
        {-9991.672727, -250.8},
        {-10095.85455, -158},
        {-10205.43636, -109.8},
        {-10315.01818, -48.8},
        {-10428.4, -20.2},
        {-10541.78182, 8.4},
        {-10652.16364, 15.8},
        {-10762.54545, 23.2},
        {-10872.92727, 13.8},
        {-10983.30909, 4.4},
        {-11096.89091, -5},
        {-11210.67273, -9},
        {-11361.65455, -48.6},
        {-11512.63636, -84.2},
        {-11785.21818, -239.4},
        {-12054.6, -394.6},
        {-12511.78182, -736.2},
        {-13159.36364, -1268.4},
        {-13806.94545, -1800.6},
        {-14332.92727, -2213.2},
        {-14858.90909, -2625.8},
        {-15440.09091, -3092.2},
        {-15793.67273, -3332.4},
        {-16377.65455, -3767.8},
        {-16961.63636, -4203.2},
        {-17768.61818, -4742.6},
        {-18332.6, -5041.8},
        {-19130.18182, -5387.6},
        {-19697.36364, -5538.2},
        {-20501.94545, -5700.4},
        {-21083.52727, -5758.6},
        {-21894.70909, -5803.8},
        {-22472.29091, -5802.4},
        {-23274.07273, -5793.8},
        {-23838.45455, -5773.6},
        {-24631.83636, -5759.4},
        {-25195.61818, -5758.2},
        {-25989, -5753.4},
        {-26558.18182, -5755.8},
        {-27347.76364, -5741.2},
        {-27908.34545, -5720.6},
        {-28678.32727, -5679.8},
        {-29218.70909, -5642.6},
        {-29967.09091, -5589.6},
        {-30495.07273, -5553.6},
        {-31232.85455, -5497.8},
        {-31761.23636, -5462.2},
        {-32499.81818, -5408},
        {-33030.4, -5369.6},
        {-33770.38182, -5319.8},
        {-34300.56364, -5289.8},
        {-35030.94545, -5250.2},
        {-35551.12727, -5229.2},
        {-36071.30909, -5208.2},
        {-36750.29091, -5149.8},
        {-37429.27273, -5091.4},
        {-38074.85455, -4999},
        {-38720.43636, -4906.6},
        {-39521.21818, -4760.4},
        {-39953.8, -4663},
        {-40534.38182, -4513.6},
        {-40948.16364, -4407.8},
        {-41361.94545, -4302},
        {-41767.92727, -4208},
        {-42173.90909, -4114},
        {-42573.09091, -4034.4},
        {-42972.27273, -3954.8},
        {-43503.05455, -3839.6},
        {-43886.43636, -3766.4},
        {-44397.41818, -3665},
        {-44767.2, -3601.2},
        {-45265.58182, -3518},
        {-45632.36364, -3470.4},
        {-46130.14545, -3406.6},
        {-46500.32727, -3371},
        {-46996.70909, -3319.8},
        {-47364.49091, -3288},
        {-47846.87273, -3239.6},
        {-48198.25455, -3207.4},
        {-48308.83636, -3207.4},
        {-48698.41818, -3169.666667},
        {-49000, -3142}
    };

    public static double [][]distalPoints = new double[][]{
        {10300, -2924},
        {10302.09091, -2924},
        {9913.981818, -3141.8},
        {9525.872727, -3359.6},
        {8902.363636, -3699.8},
        {8278.854545, -4040},
        {7414.345455, -4483},
        {6940.036364, -4708.2},
        {6215.327273, -5020},
        {5726.018182, -5209.4},
        {4962.309091, -5490.2},
        {4439.6, -5668.2},
        {3619.090909, -5936.4},
        {3048.981818, -6118},
        {2154.672727, -6389.4},
        {1534.763636, -6569.4},
        {570.2545455, -6851},
        {-96.45454545, -7042.4},
        {-1125.963636, -7347.4},
        {-1831.272727, -7562.6},
        {-2921.381818, -7887.8},
        {-3666.890909, -8111.4},
        {-4824.8, -8449.6},
        {-5619.909091, -8674.2},
        {-6857.618182, -9016},
        {-7710.527273, -9247.8},
        {-9017.636364, -9591.8},
        {-9912.345455, -9821.2},
        {-11265.85455, -10145},
        {-12176.76364, -10351.6},
        {-13547.07273, -10634.2},
        {-14463.18182, -10804.6},
        {-15379.29091, -10975},
        {-16300.8, -11103.4},
        {-17222.30909, -11231.8},
        {-18150.01818, -11308.6},
        {-19077.72727, -11385.4},
        {-20462.83636, -11464.8},
        {-21383.74545, -11491.8},
        {-22746.85455, -11504.2},
        {-23644.36364, -11492.2},
        {-24968.27273, -11447.6},
        {-25834.78182, -11400.4},
        {-27111.89091, -11300},
        {-27946.8, -11214.2},
        {-29174.30909, -11056.2},
        {-30352.01818, -10850},
        {-31529.72727, -10643.8},
        {-32296.83636, -10490.8},
        {-33063.94545, -10337.8},
        {-34095.65455, -10046.6},
        {-34750.76364, -9836.2},
        {-35668.47273, -9478.4},
        {-36586.18182, -9120.6},
        {-37746.69091, -8612},
        {-38250, -8313.8},
        {-38981.70909, -7867.6},
        {-39450.81818, -7568.8},
        {-40135.72727, -7134.4},
        {-40577.83636, -6850.8},
        {-41220.74545, -6459.8},
        {-41635.25455, -6216.8},
        {-42237.96364, -5900},
        {-42624.87273, -5718.8},
        {-43187.18182, -5470.2},
        {-43548.69091, -5329},
        {-44073.8, -5123},
        {-44410.70909, -4990.8},
        {-44747.61818, -4858.6},
        {-45060.52727, -4729.4},
        {-45514.63636, -4541},
        {-45805.14545, -4417.4},
        {-46095.65455, -4293.8},
        {-46515.16364, -4110},
        {-46783.27273, -3990.6},
        {-47026.78182, -3871.8},
        {-47270.29091, -3753},
        {-47619, -3575.6},
        {-47838.70909, -3458.4},
        {-48155.41818, -3287},
        {-48355.52727, -3174.2},
        {-48644.23636, -3008.4},
        {-48827.74545, -2901.2},
        {-49090.25455, -2744.4},
        {-49255.76364, -2641.8},
        {-49489.07273, -2490},
        {-49633.78182, -2391.2},
        {-49831.69091, -2240.8},
        {-49950.6, -2140},
        {-50085.10909, -1958.6},
        {-50151.81818, -1826.4},
        {-50218.52727, -1694.2},
        {-50201.43636, -1415.2},
        {-50184.34545, -1136.2},
        {-50141.25455, -859.4},
        {-50098.16364, -582.6},
        {-50055.07273, -305.8},
        {-50037.98182, -168.8},
        {-50020.89091, -31.8},
        {-50014.6, 43.2},
        {-50008.30909, 118.2},
        {-50002.41818, 177.4},
        {-50001.12727, 178},
        {-49999.83636, 163.6},
        {-49998.14545, 132.8},
        {-49996.45455, 91.6},
        {-49994.36364, 66.2},
        {-49995.07273, 33.4},
        {-49992.98182, 33.4},
        {-49992.75758, 7.666666667},
        {-50000, -17}
    };

    public static double []wristPoints = new double[]{
        0.521029174,
        0.521029174,
        0.521029174,
        0.521029174,
        0.503468096,
        0.503468096,
        0.503468096,
        0.503468096,
        0.503468096,
        0.503468096,
        0.503468096,
        0.503468096,
        0.503468096,
        0.503468096,
        0.503468096,
        0.503468096,
        0.503468096,
        0.503468096,
        0.503468096,
        0.503468096,
        0.503468096,
        0.503468096,
        0.503468096,
        0.503468096,
        0.49659431,
        0.49659431,
        0.49659431,
        0.49659431,
        0.49659431,
        0.49659431,
        0.49659431,
        0.49659431,
        0.49659431,
        0.49659431,
        0.49659431,
        0.49659431,
        0.49659431,
        0.49659431,
        0.49659431,
        0.49659431,
        0.49659431,
        0.49659431,
        0.49659431,
        0.49659431,
        0.495625496,
        0.495625496,
        0.495625496,
        0.495625496,
        0.495625496,
        0.495625496,
        0.495625496,
        0.495625496,
        0.495625496,
        0.495625496,
        0.495625496,
        0.495625496,
        0.495625496,
        0.495625496,
        0.495625496,
        0.495625496,
        0.495625496,
        0.495625496,
        0.495625496,
        0.495625496,
        0.440081954,
        0.440081954,
        0.440081954,
        0.440081954,
        0.440081954,
        0.440081954,
        0.440081954,
        0.440081954,
        0.440081954,
        0.440081954,
        0.440081954,
        0.440081954,
        0.440081954,
        0.440081954,
        0.440081954,
        0.440081954,
        0.440081954,
        0.440081954,
        0.440081954,
        0.440081954,
        0.367853701,
        0.367853701,
        0.367853701,
        0.367853701,
        0.367853701,
        0.367853701,
        0.367853701,
        0.367853701,
        0.367853701,
        0.367853701,
        0.367853701,
        0.367853701,
        0.367853701,
        0.367853701,
        0.367853701,
        0.367853701,
        0.367853701,
        0.367853701,
        0.367853701,
        0.367853701,
        0.375603974,
        0.375603974,
        0.375603974,
        0.375603974,
        0.375603974,
        0.375603974,
        0.375603974
    };

};