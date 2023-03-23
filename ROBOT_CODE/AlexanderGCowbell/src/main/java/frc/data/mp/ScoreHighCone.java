package frc.data.mp;

public class ScoreHighCone {
    public static ArmPath getArmPath() {
        return new ArmPath(kNumPoints, proximalPoints, distalPoints, kWristFlexIndex, kWristFlexPosition, kWristExtendIndex, kWristMaxVelocity);
    }

    public static final int kNumPoints = 100;
    public static final int kWristFlexIndex = 50;
    public static final double kWristFlexPosition = 0.426;
    public static final int kWristExtendIndex = 90;
    public static final double kWristMaxVelocity = 2000;


    public static double [][]proximalPoints = new double[][]{
        {-4388, -13},
        {-4358.385321, -13.33333333},
        {-4328.970642, -14.2},
        {-4299.555963, -15},
        {-4273.941284, -19.6},
        {-4248.326606, -24},
        {-4222.711927, -28.4},
        {-4251.497248, -81.2},
        {-4280.282569, -134},
        {-4426.46789, -303},
        {-4572.653211, -472},
        {-4852.038532, -774.4},
        {-5076.823853, -1027.8},
        {-5423.609174, -1403.6},
        {-5649.194495, -1659.4},
        {-5985.979817, -2023.2},
        {-6189.565138, -2253.6},
        {-6493.350459, -2531.6},
        {-6675.13578, -2687.2},
        {-6944.721101, -2810.8},
        {-7103.106422, -2826.4},
        {-7335.891743, -2783.8},
        {-7468.477064, -2693.6},
        {-7658.062385, -2538.2},
        {-7759.847706, -2414.8},
        {-7905.433028, -2224.2},
        {-7976.618349, -2091.8},
        {-8081.60367, -1893},
        {-8129.588991, -1759.4},
        {-8201.974312, -1561.8},
        {-8230.559633, -1431.4},
        {-8277.344954, -1244.6},
        {-8290.330275, -1124.2},
        {-8367.315596, -1008.2},
        {-8419.900917, -956.2},
        {-8616.886239, -1003},
        {-8795.67156, -1106.2},
        {-9162.056881, -1362.6},
        {-9464.442202, -1614.6},
        {-9958.027523, -2033.2},
        {-10307.21284, -2353},
        {-10850.19817, -2848.2},
        {-11205.58349, -3190.2},
        {-11777.76881, -3686.4},
        {-12158.75413, -4016},
        {-12781.93945, -4444.2},
        {-13211.32477, -4697},
        {-13901.11009, -5022.6},
        {-14374.09541, -5194},
        {-14847.08073, -5365.4},
        {-15632.66606, -5607.6},
        {-16418.25138, -5849.8},
        {-17252.2367, -6111.6},
        {-18086.22202, -6373.4},
        {-19243.40734, -6716.2},
        {-19845.79266, -6889.6},
        {-20766.17798, -7121.6},
        {-21377.7633, -7261.2},
        {-22314.54862, -7455.8},
        {-22928.13394, -7569.4},
        {-23878.11927, -7735.2},
        {-24510.10459, -7842.4},
        {-25481.48991, -7980.6},
        {-26127.67523, -8063.8},
        {-27118.26055, -8167.6},
        {-27772.44587, -8219.2},
        {-28772.43119, -8299.4},
        {-29433.01651, -8348.6},
        {-30434.20183, -8413.4},
        {-31090.98716, -8457.6},
        {-31747.77248, -8501.8},
        {-32386.9578, -8509.6},
        {-33026.14312, -8517.4},
        {-33641.92844, -8487},
        {-34257.71376, -8456.6},
        {-35178.29908, -8388.6},
        {-35770.6844, -8328.4},
        {-36665.06972, -8222.8},
        {-37242.25505, -8139.8},
        {-38113.64037, -8012},
        {-38680.22569, -7921.8},
        {-39525.61101, -7781.4},
        {-40068.99633, -7686.4},
        {-40875.18165, -7537.6},
        {-41387.16697, -7433.6},
        {-42146.35229, -7271.2},
        {-42626.73761, -7159},
        {-43337.32294, -6976},
        {-43785.10826, -6846.8},
        {-44437.29358, -6627.6},
        {-44842.2789, -6466.8},
        {-45417.86422, -6198.8},
        {-45763.24954, -6001.6},
        {-46232.83486, -5667},
        {-46498.02018, -5422.4},
        {-46853.2055, -5021.2},
        {-47037.79083, -4727.2},
        {-47293.77615, -4274.2},
        {-47425.56147, -3958.6},
        {-47616.14679, -3497},
        {-47716.73211, -3192},
        {-47856.71743, -2755},
        {-47925.30275, -2477},
        {-48011.48807, -2091.4},
        {-48038.87339, -1851.8},
        {-48069.65872, -1525.4},
        {-48061.04404, -1331},
        {-48052.42936, -1060.2},
        {-48029.61468, -835.6666667},
        {-48000, -581}
    };

    public static double [][]distalPoints = new double[][]{
        {10300, -260},
        {10076.92049, -506},
        {9857.974312, -748.8},
        {9597.761468, -1032.8},
        {9412.148624, -1243},
        {9226.53578, -1453.2},
        {8996.122936, -1696.8},
        {8765.710092, -1940.4},
        {8453.697248, -2223.6},
        {8141.684404, -2506.8},
        {7571.07156, -2972.6},
        {7142.658716, -3308.2},
        {6413.245872, -3848},
        {5879.233028, -4234.8},
        {5015.020183, -4838},
        {4409.407339, -5258.6},
        {3442.594495, -5897},
        {2776.781651, -6331.2},
        {1710.568807, -6971.4},
        {974.5559633, -7395.2},
        {-196.4568807, -7997.4},
        {-1006.269725, -8381.8},
        {-2270.082569, -8919.2},
        {-3133.495413, -9250.6},
        {-3996.908257, -9582},
        {-5343.721101, -9963},
        {-6690.533945, -10344},
        {-8034.946789, -10624.2},
        {-9379.359633, -10904.4},
        {-11165.77248, -11192.4},
        {-12033.78532, -11252.4},
        {-13332.39817, -11289.4},
        {-14179.41101, -11274.2},
        {-15467.62385, -11239.8},
        {-16313.8367, -11197.6},
        {-17607.24954, -11146.2},
        {-18470.06239, -11117.8},
        {-19766.07523, -11070.8},
        {-20620.88807, -11043},
        {-21890.50092, -10988},
        {-22712.91376, -10942.2},
        {-23934.12661, -10864.2},
        {-24722.13945, -10804.8},
        {-25890.95229, -10686.6},
        {-26644.96514, -10595.6},
        {-27764.37798, -10421.2},
        {-28484.99083, -10279},
        {-29554.00367, -10052.6},
        {-30242.21651, -9885},
        {-30930.42936, -9717.4},
        {-31586.0422, -9551.8},
        {-32241.65505, -9386.2},
        {-32877.86789, -9234.2},
        {-33514.08073, -9082.2},
        {-34481.09358, -8879.6},
        {-35115.30642, -8758.4},
        {-36085.51927, -8608.2},
        {-36726.73211, -8528.6},
        {-37700.94495, -8432},
        {-38344.3578, -8386},
        {-39310.97064, -8330.6},
        {-39941.58349, -8304.2},
        {-40885.39633, -8262.6},
        {-41496.20917, -8238},
        {-42406.02202, -8181.8},
        {-42992.63486, -8135},
        {-43863.04771, -8036.6},
        {-44420.26055, -7953.4},
        {-45246.47339, -7807.2},
        {-45773.68624, -7692.6},
        {-46300.89908, -7578},
        {-47036.91193, -7371.2},
        {-47772.92477, -7164.4},
        {-48442.73761, -6925.4},
        {-49112.55046, -6686.4},
        {-49935.1633, -6317.2},
        {-50265.17615, -6091.8},
        {-50716.18899, -5718.8},
        {-50964.40183, -5441},
        {-51318.41468, -5013.8},
        {-51519.62752, -4716.8},
        {-51814.64037, -4275.4},
        {-51988.65321, -3981.6},
        {-52162.66606, -3687.8},
        {-52307.0789, -3416.2},
        {-52451.49174, -3144.6},
        {-52556.70459, -2918.2},
        {-52661.91743, -2691.8},
        {-52806.33028, -2383.4},
        {-52874.54312, -2202.2},
        {-52973.15596, -1945.8},
        {-53017.16881, -1788.6},
        {-53085.38165, -1561.6},
        {-53114.3945, -1416.6},
        {-53161.40734, -1212.8},
        {-53178.02018, -1084.2},
        {-53205.83303, -912},
        {-53209.44587, -809.6},
        {-53220.05872, -675},
        {-53212.67156, -599.2},
        {-53207.4844, -494.8},
        {-53191.09725, -434},
        {-53174.71009, -347.6},
        {-53151.32294, -293.4},
        {-53127.93578, -239.2},
        {-53102.34862, -179},
        {-53076.76147, -118.8},
        {-53051.17431, -60.4},
        {-53025.58716, 24.66666667},
        {-53000, 104}
    };

    public static double []wristPoints = new double[]{
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
        0.553058267,
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
        0.468208671
    };

};
