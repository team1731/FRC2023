package frc.data.mp;

public class PickupHighCubeExtra {
    public static ArmPath getArmPath() {
        return new ArmPath(kNumPoints, proximalPoints, distalPoints, kWristFlexIndex, kWristFlexPosition, kWristExtendIndex, kWristMaxVelocity);
    }

    public static final int kNumPoints = 212;
    public static final int kWristFlexIndex = 75;
    public static final double kWristFlexPosition = 0.123;
    public static final int kWristExtendIndex = 210;
    public static final double kWristMaxVelocity = 2000;

    // Position (ticks)	Velocity (RPM)

   

    public static double [][]proximalPoints = new double[][]{
        {-11863.0,  -3364.0},
        {-11863.0,  -3364.0},
        {-12662.0,  -3590.0},
        {-12662.0,  -3590.0},
        {-12662.0,  -3590.0},
        {-12662.0,  -3590.0},
        {-13485.0,  -3802.0},
        {-13485.0,  -3802.0},
        {-13485.0,  -3802.0},
        {-13485.0,  -3802.0},
        {-15237.0,  -4154.0},
        {-15237.0,  -4154.0},
        {-15237.0,  -4154.0},
        {-15237.0,  -4154.0},
        {-16143.0,  -4277.0},
        {-16143.0,  -4277.0},
        {-16143.0,  -4277.0},
        {-16143.0,  -4277.0},
        {-17089.0,  -4424.0},
        {-17089.0,  -4424.0},
        {-18092.0,  -4602.0},
        {-18092.0,  -4602.0},
        {-19137.0,  -4781.0},
        {-19137.0,  -4781.0},
        {-20207.0,  -4964.0},
        {-20207.0,  -4964.0},
        {-21277.0,  -5131.0},
        {-21277.0,  -5131.0},
        {-22339.0,  -5245.0},
        {-22339.0,  -5245.0},
        {-23411.0,  -5317.0},
        {-23411.0,  -5317.0},
        {-24506.0,  -5366.0},
        {-24506.0,  -5366.0},
        {-25602.0,  -5396.0},
        {-25602.0,  -5396.0},
        {-26685.0,  -5406.0},
        {-26685.0,  -5406.0},
        {-26685.0,  -5406.0},
        {-27741.0,  -5402.0},
        {-27741.0,  -5402.0},
        {-28787.0,  -5377.0},
        {-28787.0,  -5377.0},
        {-29825.0,  -5322.0},
        {-29825.0,  -5322.0},
        {-30830.0,  -5231.0},
        {-30830.0,  -5231.0},
        {-31798.0,  -5115.0},
        {-31798.0,  -5115.0},
        {-32717.0,  -4980.0},
        {-32717.0,  -4980.0},
        {-33587.0,  -4803.0},
        {-33587.0,  -4803.0},
        {-33587.0,  -4803.0},
        {-35203.0,  -4378.0},
        {-35203.0,  -4378.0},
        {-35960.0,  -4171.0},
        {-35960.0,  -4171.0},
        {-36701.0,  -3991.0},
        {-36701.0,  -3991.0},
        {-37448.0,  -3865.0},
        {-37448.0,  -3865.0},
        {-38171.0,  -3763.0},
        {-38171.0,  -3763.0},
        {-38845.0,  -3648.0},
        {-38845.0,  -3648.0},
        {-39478.0,  -3522.0},
        {-39478.0,  -3522.0},
        {-40059.0,  -3364.0},
        {-40059.0,  -3364.0},
        {-40059.0,  -3364.0},
        {-40627.0,  -3186.0},
        {-40627.0,  -3186.0},
        {-41190.0,  -3023.0},
        {-41190.0,  -3023.0},
        {-41743.0,  -2900.0},
        {-41743.0,  -2900.0},
        {-42289.0,  -2813.0},
        {-42289.0,  -2813.0},
        {-42805.0,  -2748.0},
        {-42805.0,  -2748.0},
        {-43304.0,  -2680.0},
        {-43304.0,  -2680.0},
        {-43777.0,  -2591.0},
        {-43777.0,  -2591.0},
        {-44232.0,  -2491.0},
        {-44232.0,  -2491.0},
        {-44650.0,  -2365.0},
        {-44650.0,  -2365.0},
        {-45037.0,  -2236.0},
        {-45037.0,  -2236.0},
        {-45418.0,  -2116.0},
        {-45418.0,  -2116.0},
        {-45777.0,  -2003.0},
        {-45777.0,  -2003.0},
        {-46114.0,  -1887.0},
        {-46114.0,  -1887.0},
        {-46425.0,  -1779.0},
        {-46425.0,  -1779.0},
        {-46706.0,  -1673.0},
        {-46706.0,  -1673.0},
        {-46955.0,  -1543.0},
        {-46955.0,  -1543.0},
        {-47172.0,  -1401.0},
        {-47172.0,  -1401.0},
        {-47354.0,  -1246.0},
        {-47354.0,  -1246.0},
        {-47501.0,  -1082.0},
        {-47501.0,  -1082.0},
        {-47613.0,  -911.0},
        {-47613.0,  -911.0},
        {-47683.0,  -734.0},
        {-47683.0,  -734.0},
        {-47713.0,  -547.0},
        {-47713.0,  -547.0},
        {-47686.0,  -338.0},
        {-47686.0,  -338.0},
        {-47686.0,  -338.0},
        {-47695.0,  -86.0},
        {-47695.0,  -86.0},
        {-47692.0,  -11.0},
        {-47692.0,  -11.0},
        {-47692.0,  21.0},
        {-47692.0,  21.0},
        {-47692.0,  -5.0},
        {-47692.0,  -5.0},
        {-47692.0,  -7.0},
        {-47692.0,  -7.0},
        {-47691.0,  3.0},
        {-47691.0,  3.0},
        {-47681.0,  11.0},
        {-47681.0,  11.0},
        {-47674.0,  18.0},
        {-47674.0,  18.0},
        {-47674.0,  18.0},
        {-47672.0,  20.0},
        {-47672.0,  20.0},
        {-47607.0,  79.0},
        {-47607.0,  79.0},
        {-47322.0,  359.0},
        {-47322.0,  359.0},
        {-46926.0,  740.0},
        {-46926.0,  740.0},
        {-46553.0,  1109.0},
        {-46553.0,  1109.0},
        {-46249.0,  1413.0},
        {-46249.0,  1413.0},
        {-45992.0,  1613.0},
        {-45992.0,  1613.0},
        {-45745.0,  1579.0},
        {-45745.0,  1579.0},
        {-45426.0,  1503.0},
        {-45426.0,  1503.0},
        {-45014.0,  1536.0},
        {-45014.0,  1536.0},
        {-44533.0,  1710.0},
        {-44533.0,  1710.0},
        {-44034.0,  1949.0},
        {-44034.0,  1949.0},
        {-43565.0,  2174.0},
        {-43565.0,  2174.0},
        {-43112.0,  2311.0},
        {-43112.0,  2311.0},
        {-42664.0,  2351.0},
        {-42664.0,  2351.0},
        {-42188.0,  2344.0},
        {-42188.0,  2344.0},
        {-41685.0,  2350.0},
        {-41685.0,  2350.0},
        {-41183.0,  2382.0},
        {-41183.0,  2382.0},
        {-40690.0,  2422.0},
        {-40690.0,  2422.0},
        {-40228.0,  2436.0},
        {-40228.0,  2436.0},
        {-39796.0,  2395.0},
        {-39796.0,  2395.0},
        {-39408.0,  2281.0},
        {-39408.0,  2281.0},
        {-39029.0,  2155.0},
        {-39029.0,  2155.0},
        {-38642.0,  2050.0},
        {-38642.0,  2050.0},
        {-38246.0,  1983.0},
        {-38246.0,  1983.0},
        {-37853.0,  1944.0},
        {-37853.0,  1944.0},
        {-37853.0,  1944.0},
        {-37164.0,  1868.0},
        {-37164.0,  1868.0},
        {-36860.0,  1785.0},
        {-36860.0,  1785.0},
        {-36591.0,  1661.0},
        {-36591.0,  1661.0},
        {-36360.0,  1499.0},
        {-36360.0,  1499.0},
        {-36186.0,  1316.0},
        {-36186.0,  1316.0},
        {-36186.0,  1316.0},
        {-36057.0,  1114.0},
        {-36057.0,  1114.0},
        {-35944.0,  922.0},
        {-35944.0,  922.0},
        {-35839.0,  756.0},
        {-35839.0,  756.0},
        {-35750.0,  614.0},
        {-35750.0,  614.0},
        {-35671.0,  517.0},
        {-35671.0,  517.0},
        {-35651.0,  410.0},
        {-35651.0,  410.0},
        {-35651.0,  297.0},
        {-35651.0,  297.0},
        {-35651.0,  191.0},
        {-35651.0,  191.0},
        {-35651.0,  102.0},
        {-35651.0,  102.0},
        {-35651.0,  22.0}
    };

    public static double [][]distalPoints = new double[][]{
        { 12698.0,  1639.0},
        { 12698.0,  1639.0},
        { 13208.0,  1882.0},
        { 13208.0,  1882.0},
        { 13208.0,  1882.0},
        { 13208.0,  1882.0},
        { 13771.0,  2149.0},
        { 13771.0,  2149.0},
        { 13771.0,  2149.0},
        { 13771.0,  2149.0},
        { 14392.0,  2458.0},
        { 14392.0,  2458.0},
        { 14392.0,  2458.0},
        { 14392.0,  2458.0},
        { 15064.0,  2784.0},
        { 15064.0,  2784.0},
        { 15064.0,  2784.0},
        { 15064.0,  2784.0},
        { 15812.0,  3103.0},
        { 15812.0,  3103.0},
        { 16656.0,  3436.0},
        { 16656.0,  3436.0},
        { 17570.0,  3787.0},
        { 17570.0,  3787.0},
        { 18525.0,  4124.0},
        { 18525.0,  4124.0},
        { 19535.0,  4459.0},
        { 19535.0,  4459.0},
        { 20621.0,  4799.0},
        { 20621.0,  4799.0},
        { 21768.0,  5103.0},
        { 21768.0,  5103.0},
        { 22946.0,  5367.0},
        { 22946.0,  5367.0},
        { 24178.0,  5646.0},
        { 24178.0,  5646.0},
        { 26689.0,  6063.0},
        { 26689.0,  6063.0},
        { 26689.0,  6063.0},
        { 27945.0,  6174.0},
        { 27945.0,  6174.0},
        { 29182.0,  6237.0},
        { 29182.0,  6237.0},
        { 30404.0,  6226.0},
        { 30404.0,  6226.0},
        { 31586.0,  6157.0},
        { 31586.0,  6157.0},
        { 31586.0,  6157.0},
        { 32756.0,  6070.0},
        { 32756.0,  6070.0},
        { 33916.0,  5974.0},
        { 33916.0,  5974.0},
        { 36139.0,  5741.0},
        { 36139.0,  5741.0},
        { 36139.0,  5741.0},
        { 37179.0,  5600.0},
        { 37179.0,  5600.0},
        { 38163.0,  5415.0},
        { 38163.0,  5415.0},
        { 39092.0,  5187.0},
        { 39092.0,  5187.0},
        { 39969.0,  4929.0},
        { 39969.0,  4929.0},
        { 40788.0,  4656.0},
        { 40788.0,  4656.0},
        { 41561.0,  4386.0},
        { 41561.0,  4386.0},
        { 41561.0,  4386.0},
        { 42313.0,  4156.0},
        { 42313.0,  4156.0},
        { 43062.0,  3974.0},
        { 43062.0,  3974.0},
        { 43806.0,  3844.0},
        { 43806.0,  3844.0},
        { 44535.0,  3749.0},
        { 44535.0,  3749.0},
        { 45224.0,  3669.0},
        { 45224.0,  3669.0},
        { 45897.0,  3585.0},
        { 45897.0,  3585.0},
        { 47262.0,  3457.0},
        { 47262.0,  3457.0},
        { 47262.0,  3457.0},
        { 47952.0,  3419.0},
        { 47952.0,  3419.0},
        { 48648.0,  3423.0},
        { 48648.0,  3423.0},
        { 49359.0,  3461.0},
        { 49359.0,  3461.0},
        { 50090.0,  3514.0},
        { 50090.0,  3514.0},
        { 50835.0,  3572.0},
        { 50835.0,  3572.0},
        { 51571.0,  3616.0},
        { 51571.0,  3616.0},
        { 52303.0,  3653.0},
        { 52303.0,  3653.0},
        { 53031.0,  3671.0},
        { 53031.0,  3671.0},
        { 53747.0,  3658.0},
        { 53747.0,  3658.0},
        { 54461.0,  3627.0},
        { 54461.0,  3627.0},
        { 55156.0,  3587.0},
        { 55156.0,  3587.0},
        { 55812.0,  3511.0},
        { 55812.0,  3511.0},
        { 56432.0,  3405.0},
        { 56432.0,  3405.0},
        { 57008.0,  3265.0},
        { 57008.0,  3265.0},
        { 57538.0,  3083.0},
        { 57538.0,  3083.0},
        { 58027.0,  2879.0},
        { 58027.0,  2879.0},
        { 58480.0,  2676.0},
        { 58480.0,  2676.0},
        { 58480.0,  2676.0},
        { 58904.0,  2480.0},
        { 58904.0,  2480.0},
        { 59317.0,  2314.0},
        { 59317.0,  2314.0},
        { 59728.0,  2193.0},
        { 59728.0,  2193.0},
        { 60140.0,  2114.0},
        { 60140.0,  2114.0},
        { 60527.0,  2048.0},
        { 60527.0,  2048.0},
        { 60889.0,  1987.0},
        { 60889.0,  1987.0},
        { 61247.0,  1933.0},
        { 61247.0,  1933.0},
        { 62011.0,  1871.0},
        { 62011.0,  1871.0},
        { 62011.0,  1871.0},
        { 62418.0,  1891.0},
        { 62418.0,  1891.0},
        { 62829.0,  1938.0},
        { 62829.0,  1938.0},
        { 63244.0,  1994.0},
        { 63244.0,  1994.0},
        { 63640.0,  2022.0},
        { 63640.0,  2022.0},
        { 64001.0,  1993.0},
        { 64001.0,  1993.0},
        { 64326.0,  1911.0},
        { 64326.0,  1911.0},
        { 64613.0,  1788.0},
        { 64613.0,  1788.0},
        { 64883.0,  1644.0},
        { 64883.0,  1644.0},
        { 65191.0,  1552.0},
        { 65191.0,  1552.0},
        { 65534.0,  1534.0},
        { 65534.0,  1534.0},
        { 65859.0,  1533.0},
        { 65859.0,  1533.0},
        { 66140.0,  1528.0},
        { 66140.0,  1528.0},
        { 66371.0,  1490.0},
        { 66371.0,  1490.0},
        { 66554.0,  1368.0},
        { 66554.0,  1368.0},
        { 66693.0,  1165.0},
        { 66693.0,  1165.0},
        { 66791.0,  940.0},
        { 66791.0,  940.0},
        { 66857.0,  723.0},
        { 66857.0,  723.0},
        { 66893.0,  529.0},
        { 66893.0,  529.0},
        { 66881.0,  334.0},
        { 66881.0,  334.0},
        { 66872.0,  183.0},
        { 66872.0,  183.0},
        { 66872.0,  83.0},
        { 66872.0,  83.0},
        { 66872.0,  17.0},
        { 66872.0,  17.0},
        { 66872.0,  -21.0},
        { 66872.0,  -21.0},
        { 66870.0,  -11.0},
        { 66870.0,  -11.0},
        { 66834.0,  -36.0},
        { 66834.0,  -36.0},
        { 66723.0,  -144.0},
        { 66723.0,  -144.0},
        { 66723.0,  -144.0},
        { 66558.0,  -309.0},
        { 66558.0,  -309.0},
        { 66413.0,  -454.0},
        { 66413.0,  -454.0},
        { 66304.0,  -564.0},
        { 66304.0,  -564.0},
        { 66228.0,  -606.0},
        { 66228.0,  -606.0},
        { 66146.0,  -416.0},
        { 66146.0,  -416.0},
        { 66146.0,  -416.0},
        { 66127.0,  -291.0},
        { 66127.0,  -291.0},
        { 66132.0,  -175.0},
        { 66132.0,  -175.0},
        { 66134.0,  -96.0},
        { 66134.0,  -96.0},
        { 66134.0,  -49.0},
        { 66134.0,  -49.0},
        { 66134.0,  -13.0},
        { 66134.0,  -13.0},
        { 66134.0,  7.0},
        { 66134.0,  7.0},
        { 66134.0,  2.0},
        { 66134.0,  2.0},
        { 66134.0,  0.0},
        { 66134.0,  0.0},
        { 66134.0,  0.0},
        { 66134.0,  0.0},
        { 66134.0,  0.0}
    };

    public static double []wristPoints = new double[]{
         0.4965851604938507,
         0.4965851604938507,
         0.4965851604938507,
         0.4965851604938507,
         0.4965851604938507,
         0.4965851604938507,
         0.4965851604938507,
         0.4965851604938507,
         0.4965851604938507,
         0.4965851604938507,
         0.4965851604938507,
         0.4965851604938507,
         0.4965851604938507,
         0.4965851604938507,
         0.4965851604938507,
         0.4965851604938507,
         0.4965851604938507,
         0.4965851604938507,
         0.4965851604938507,
         0.49765750765800476,
         0.49765750765800476,
         0.49765750765800476,
         0.49765750765800476,
         0.49765750765800476,
         0.49765750765800476,
         0.49765750765800476,
         0.49765750765800476,
         0.49765750765800476,
         0.49765750765800476,
         0.49765750765800476,
         0.49765750765800476,
         0.49765750765800476,
         0.49765750765800476,
         0.49765750765800476,
         0.49765750765800476,
         0.49765750765800476,
         0.49765750765800476,
         0.49765750765800476,
         0.49765750765800476,
         0.47600486874580383,
         0.47600486874580383,
         0.47600486874580383,
         0.47600486874580383,
         0.47600486874580383,
         0.47600486874580383,
         0.47600486874580383,
         0.47600486874580383,
         0.47600486874580383,
         0.47600486874580383,
         0.47600486874580383,
         0.47600486874580383,
         0.47600486874580383,
         0.47600486874580383,
         0.47600486874580383,
         0.47600486874580383,
         0.47600486874580383,
         0.47600486874580383,
         0.47600486874580383,
         0.47600486874580383,
         0.43793925642967224,
         0.43793925642967224,
         0.43793925642967224,
         0.43793925642967224,
         0.43793925642967224,
         0.43793925642967224,
         0.43793925642967224,
         0.43793925642967224,
         0.43793925642967224,
         0.43793925642967224,
         0.43793925642967224,
         0.43793925642967224,
         0.43793925642967224,
         0.43793925642967224,
         0.43793925642967224,
         0.43793925642967224,
         0.43793925642967224,
         0.43793925642967224,
         0.43793925642967224,
         0.43793925642967224,
         0.39708569645881653,
         0.39708569645881653,
         0.39708569645881653,
         0.39708569645881653,
         0.39708569645881653,
         0.39708569645881653,
         0.39708569645881653,
         0.39708569645881653,
         0.39708569645881653,
         0.39708569645881653,
         0.39708569645881653,
         0.39708569645881653,
         0.39708569645881653,
         0.39708569645881653,
         0.39708569645881653,
         0.39708569645881653,
         0.39708569645881653,
         0.39708569645881653,
         0.39708569645881653,
         0.39708569645881653,
         0.3415659964084625,
         0.3415659964084625,
         0.3415659964084625,
         0.3415659964084625,
         0.3415659964084625,
         0.3415659964084625,
         0.3415659964084625,
         0.3415659964084625,
         0.3415659964084625,
         0.3415659964084625,
         0.3415659964084625,
         0.3415659964084625,
         0.3415659964084625,
         0.3415659964084625,
         0.3415659964084625,
         0.3415659964084625,
         0.3415659964084625,
         0.3415659964084625,
         0.3415659964084625,
         0.3415659964084625,
         0.2858138978481293,
         0.2858138978481293,
         0.2858138978481293,
         0.2858138978481293,
         0.2858138978481293,
         0.2858138978481293,
         0.2858138978481293,
         0.2858138978481293,
         0.2858138978481293,
         0.2858138978481293,
         0.2858138978481293,
         0.2858138978481293,
         0.2858138978481293,
         0.2858138978481293,
         0.2858138978481293,
         0.2858138978481293,
         0.2858138978481293,
         0.2858138978481293,
         0.2858138978481293,
         0.2858138978481293,
         0.23695877194404602,
         0.23695877194404602,
         0.23695877194404602,
         0.23695877194404602,
         0.23695877194404602,
         0.23695877194404602,
         0.23695877194404602,
         0.23695877194404602,
         0.23695877194404602,
         0.23695877194404602,
         0.23695877194404602,
         0.23695877194404602,
         0.23695877194404602,
         0.23695877194404602,
         0.23695877194404602,
         0.23695877194404602,
         0.23695877194404602,
         0.23695877194404602,
         0.23695877194404602,
         0.23695877194404602,
         0.16095104813575745,
         0.16095104813575745,
         0.16095104813575745,
         0.16095104813575745,
         0.16095104813575745,
         0.16095104813575745,
         0.16095104813575745,
         0.16095104813575745,
         0.16095104813575745,
         0.16095104813575745,
         0.16095104813575745,
         0.16095104813575745,
         0.16095104813575745,
         0.16095104813575745,
         0.16095104813575745,
         0.16095104813575745,
         0.16095104813575745,
         0.16095104813575745,
         0.16095104813575745,
         0.16095104813575745,
         0.1277780830860138,
         0.1277780830860138,
         0.1277780830860138,
         0.1277780830860138,
         0.1277780830860138,
         0.1277780830860138,
         0.1277780830860138,
         0.1277780830860138,
         0.1277780830860138,
         0.1277780830860138,
         0.1277780830860138,
         0.1277780830860138,
         0.1277780830860138,
         0.1277780830860138,
         0.1277780830860138,
         0.1277780830860138,
         0.1277780830860138,
         0.1277780830860138,
         0.1277780830860138,
         0.1277780830860138,
         0.12385842204093933,
         0.12385842204093933,
         0.12385842204093933,
         0.12385842204093933,
         0.12385842204093933,
         0.12385842204093933,
         0.12385842204093933,
         0.12385842204093933,
         0.12385842204093933,
         0.12385842204093933,
         0.12385842204093933,
         0.12385842204093933,
         0.12385842204093933,
         0.12385842204093933,
         0.12385842204093933,
         0.12385842204093933,
         0.12385842204093933,
         0.12385842204093933,
         0.12385842204093933
    };

};
