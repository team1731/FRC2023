package frc.data.mp;

public class ScoreHighCone {
    public static ArmPath getArmPath() {
        return new ArmPath(kNumPoints, proximalPoints, distalPoints, kWristFlexIndex, kWristFlexPosition, kWristExtendIndex, kWristMaxVelocity);
    }

    public static final int kNumPoints = 100;
    public static final int kWristFlexIndex = 50;
    public static final double kWristFlexPosition = 0.50;
    public static final int kWristExtendIndex = 90;
    public static final double kWristMaxVelocity = 2000;


    public static double [][]proximalPoints = new double[][]{
        {-4388, -13},
        {-4257.46789, -13.33333333},
        {-4127.13578, -14.2},
        {-3996.80367, -15},
        {-3870.27156, -19.6},
        {-3743.73945, -24},
        {-3617.207339, -28.4},
        {-3545.075229, -81.2},
        {-3472.943119, -134},
        {-3518.211009, -303},
        {-3563.478899, -472},
        {-3741.946789, -774.4},
        {-3865.814679, -1027.8},
        {-4111.682569, -1403.6},
        {-4236.350459, -1659.4},
        {-4472.218349, -2023.2},
        {-4574.886239, -2253.6},
        {-4777.754128, -2531.6},
        {-4858.622018, -2687.2},
        {-5027.289908, -2810.8},
        {-5084.757798, -2826.4},
        {-5216.625688, -2783.8},
        {-5248.293578, -2693.6},
        {-5336.961468, -2538.2},
        {-5337.829358, -2414.8},
        {-5382.497248, -2224.2},
        {-5352.765138, -2091.8},
        {-5356.833028, -1893},
        {-5303.900917, -1759.4},
        {-5275.368807, -1561.8},
        {-5203.036697, -1431.4},
        {-5148.904587, -1244.6},
        {-5060.972477, -1124.2},
        {-5037.040367, -1008.2},
        {-4988.708257, -956.2},
        {-5084.776147, -1003},
        {-5162.644037, -1106.2},
        {-5428.111927, -1362.6},
        {-5629.579817, -1614.6},
        {-6022.247706, -2033.2},
        {-6270.515596, -2353},
        {-6712.583486, -2848.2},
        {-6967.051376, -3190.2},
        {-7438.319266, -3686.4},
        {-7718.387156, -4016},
        {-8240.655046, -4444.2},
        {-8569.122936, -4697},
        {-9157.990826, -5022.6},
        {-9530.058716, -5194},
        {-9902.126606, -5365.4},
        {-10586.7945, -5607.6},
        {-11271.46239, -5849.8},
        {-12004.53028, -6111.6},
        {-12737.59817, -6373.4},
        {-13793.86606, -6716.2},
        {-14295.33394, -6889.6},
        {-15114.80183, -7121.6},
        {-15625.46972, -7261.2},
        {-16461.33761, -7455.8},
        {-16974.0055, -7569.4},
        {-17823.07339, -7735.2},
        {-18354.14128, -7842.4},
        {-19224.60917, -7980.6},
        {-19769.87706, -8063.8},
        {-20659.54495, -8167.6},
        {-21212.81284, -8219.2},
        {-22111.88073, -8299.4},
        {-22671.54862, -8348.6},
        {-23571.81651, -8413.4},
        {-24127.6844, -8457.6},
        {-24683.55229, -8501.8},
        {-25221.82018, -8509.6},
        {-25760.08807, -8517.4},
        {-26274.95596, -8487},
        {-26789.82385, -8456.6},
        {-27609.49174, -8388.6},
        {-28100.95963, -8328.4},
        {-28894.42752, -8222.8},
        {-29370.69541, -8139.8},
        {-30141.1633, -8012},
        {-30606.83119, -7921.8},
        {-31351.29908, -7781.4},
        {-31793.76697, -7686.4},
        {-32499.03486, -7537.6},
        {-32910.10275, -7433.6},
        {-33568.37064, -7271.2},
        {-33947.83853, -7159},
        {-34557.50642, -6976},
        {-34904.37431, -6846.8},
        {-35455.6422, -6627.6},
        {-35759.71009, -6466.8},
        {-36234.37798, -6198.8},
        {-36478.84587, -6001.6},
        {-36847.51376, -5667},
        {-37011.78165, -5422.4},
        {-37266.04954, -5021.2},
        {-37349.71743, -4727.2},
        {-37504.78532, -4274.2},
        {-37535.65321, -3958.6},
        {-37625.3211, -3497},
        {-37624.98899, -3192},
        {-37664.05688, -2755},
        {-37631.72477, -2477},
        {-37616.99266, -2091.4},
        {-37543.46055, -1851.8},
        {-37473.32844, -1525.4},
        {-37363.79633, -1331},
        {-37254.26422, -1060.2},
        {-37130.53211, -835.6666667},
        {-37000, -581}
    };

    public static double [][]distalPoints = new double[][]{
        {10300, -260},
        {10150.31498, -506},
        {10004.7633, -748.8},
        {9817.944954, -1032.8},
        {9705.726606, -1243},
        {9593.508257, -1453.2},
        {9436.489908, -1696.8},
        {9279.47156, -1940.4},
        {9040.853211, -2223.6},
        {8802.234862, -2506.8},
        {8305.016514, -2972.6},
        {7949.998165, -3308.2},
        {7293.979817, -3848},
        {6833.361468, -4234.8},
        {6042.543119, -4838},
        {5510.324771, -5258.6},
        {4616.906422, -5897},
        {4024.488073, -6331.2},
        {3031.669725, -6971.4},
        {2369.051376, -7395.2},
        {1271.433028, -7997.4},
        {535.0146789, -8381.8},
        {-655.4036697, -8919.2},
        {-1445.422018, -9250.6},
        {-2235.440367, -9582},
        {-3508.858716, -9963},
        {-4782.277064, -10344},
        {-6053.295413, -10624.2},
        {-7324.313761, -10904.4},
        {-9037.33211, -11192.4},
        {-9831.950459, -11252.4},
        {-11057.16881, -11289.4},
        {-11830.78716, -11274.2},
        {-13045.6055, -11239.8},
        {-13818.42385, -11197.6},
        {-15038.4422, -11146.2},
        {-15827.86055, -11117.8},
        {-17050.4789, -11070.8},
        {-17831.89725, -11043},
        {-19028.1156, -10988},
        {-19777.13394, -10942.2},
        {-20924.95229, -10864.2},
        {-21639.57064, -10804.8},
        {-22734.98899, -10686.6},
        {-23415.60734, -10595.6},
        {-24461.62569, -10421.2},
        {-25108.84404, -10279},
        {-26104.46239, -10052.6},
        {-26719.28073, -9885},
        {-27334.09908, -9717.4},
        {-27916.31743, -9551.8},
        {-28498.53578, -9386.2},
        {-29061.35413, -9234.2},
        {-29624.17248, -9082.2},
        {-30517.79083, -8879.6},
        {-31078.60917, -8758.4},
        {-31975.42752, -8608.2},
        {-32543.24587, -8528.6},
        {-33444.06422, -8432},
        {-34014.08257, -8386},
        {-34907.30092, -8330.6},
        {-35464.51927, -8304.2},
        {-36334.93761, -8262.6},
        {-36872.35596, -8238},
        {-37708.77431, -8181.8},
        {-38221.99266, -8135},
        {-39019.01101, -8036.6},
        {-39502.82936, -7953.4},
        {-40255.64771, -7807.2},
        {-40709.46606, -7692.6},
        {-41163.2844, -7578},
        {-41825.90275, -7371.2},
        {-42488.5211, -7164.4},
        {-43084.93945, -6925.4},
        {-43681.3578, -6686.4},
        {-44430.57615, -6317.2},
        {-44687.1945, -6091.8},
        {-45064.81284, -5718.8},
        {-45239.63119, -5441},
        {-45520.24954, -5013.8},
        {-45648.06789, -4716.8},
        {-45869.68624, -4275.4},
        {-45970.30459, -3981.6},
        {-46070.92294, -3687.8},
        {-46141.94128, -3416.2},
        {-46212.95963, -3144.6},
        {-46244.77798, -2918.2},
        {-46276.59633, -2691.8},
        {-46347.61468, -2383.4},
        {-46342.43303, -2202.2},
        {-46367.65138, -1945.8},
        {-46338.26972, -1788.6},
        {-46333.08807, -1561.6},
        {-46288.70642, -1416.6},
        {-46262.32477, -1212.8},
        {-46205.54312, -1084.2},
        {-46159.96147, -912},
        {-46090.17982, -809.6},
        {-46027.39817, -675},
        {-45946.61651, -599.2},
        {-45868.03486, -494.8},
        {-45778.25321, -434},
        {-45688.47156, -347.6},
        {-45591.68991, -293.4},
        {-45494.90826, -239.2},
        {-45395.92661, -179},
        {-45296.94495, -118.8},
        {-45197.9633, -60.4},
        {-45098.98165, 24.66666667},
        {-45000, 104}
    };

    public static double []wristPoints = new double[]{
        0.554134011,
        0.554134011,
        0.554134011,
        0.554134011,
        0.554134011,
        0.554134011,
        0.554134011,
        0.554134011,
        0.554134011,
        0.554134011,
        0.554134011,
        0.554134011,
        0.554134011,
        0.554134011,
        0.551195025,
        0.551195025,
        0.551195025,
        0.551195025,
        0.551195025,
        0.551195025,
        0.551195025,
        0.551195025,
        0.551195025,
        0.551195025,
        0.551195025,
        0.551195025,
        0.551195025,
        0.551195025,
        0.551195025,
        0.551195025,
        0.551195025,
        0.551195025,
        0.551195025,
        0.551195025,
        0.520893931,
        0.520893931,
        0.520893931,
        0.520893931,
        0.520893931,
        0.520893931,
        0.520893931,
        0.520893931,
        0.520893931,
        0.520893931,
        0.520893931,
        0.520893931,
        0.520893931,
        0.520893931,
        0.520893931,
        0.520893931,
        0.520893931,
        0.520893931,
        0.520893931,
        0.520893931,
        0.501394033,
        0.501394033,
        0.501394033,
        0.501394033,
        0.501394033,
        0.501394033,
        0.501394033,
        0.501394033,
        0.501394033,
        0.501394033,
        0.501394033,
        0.501394033,
        0.501394033,
        0.501394033,
        0.501394033,
        0.501394033,
        0.501394033,
        0.501394033,
        0.501394033,
        0.501394033,
        0.468208671,
        0.468208671,
        0.468208671,
        0.468208671,
        0.468208671,
        0.468208671,
        0.468208671,
        0.468208671,
        0.468208671,
        0.468208671,
        0.468208671,
        0.468208671,
        0.468208671,
        0.468208671,
        0.468208671,
        0.468208671,
        0.468208671,
        0.468208671,
        0.468208671,
        0.468208671,
        0.437977374,
        0.437977374,
        0.437977374,
        0.437977374,
        0.437977374,
        0.437977374,
        0.437977374,
        0.437977374,
        0.437977374,
        0.437977374,
        0.437977374,
        0.437977374,
        0.437977374,
        0.437977374,
        0.437977374,
        0.437977374
    };

};
