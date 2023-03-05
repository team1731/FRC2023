package frc.data.mp;

public class PickupLowCube {
    public static ArmPath getArmPath() {
        return new ArmPath(kNumPoints, proximalPoints, distalPoints, kWristFlexIndex, kWristFlexPosition, kWristExtendIndex, kWristMaxVelocity);
    }

    public static final int kNumPoints = 90;
    public static final int kWristFlexIndex = 5;
    public static final double kWristFlexPosition = 0.33;
    public static final int kWristExtendIndex = 89;
    public static final double kWristMaxVelocity = 2000;

    // Position (ticks)	Velocity (RPM)

   
    public static double [][]proximalPoints = new double[][]{
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6688, 0},
        {-6689, -1},
        {-6689, -1},
        {-6689, -1},
        {-6689, -1},
        {-6689, -1},
        {-6689, -1},
        {-6689, -1},
        {-6689, -1},
        {-6689, -1},
        {-6689, -1},
        {-6689, -1},
        {-6689, -1},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6689, 0},
        {-6690, -1},
        {-6690, -1},
        {-6690, -2},
        {-6691, -2},
        {-6691, -2},
        {-6691, -2},
        {-6691, -2},
        {-6691, -2},
        {-6691, -2},
        {-6691, -2},
        {-6691, -1},
        {-6691, -1},
        {-6691, -1},
        {-6691, 0},
        {-6691, 0},
        {-6691, 0},
        {-6691, 0}
    };

    public static double [][]distalPoints = new double[][]{
        {12705, 6},
        {12711, 12},
        {12711, 12},
        {12711, 12},
        {12729, 29},
        {12729, 29},
        {12729, 29},
        {12729, 29},
        {12799, 96},
        {12799, 96},
        {12799, 96},
        {12799, 96},
        {12799, 96},
        {12799, 96},
        {12799, 96},
        {12799, 96},
        {12888, 184},
        {12888, 184},
        {12992, 282},
        {12992, 282},
        {13142, 424},
        {13142, 424},
        {13324, 589},
        {13324, 589},
        {13664, 774},
        {13664, 774},
        {13834, 839},
        {13834, 839},
        {13834, 839},
        {14051, 907},
        {14051, 907},
        {14283, 958},
        {14283, 958},
        {14500, 991},
        {14500, 991},
        {14724, 1056},
        {14724, 1056},
        {14970, 1134},
        {14970, 1134},
        {15223, 1172},
        {15223, 1172},
        {15473, 1189},
        {15473, 1189},
        {15731, 1229},
        {15731, 1229},
        {16008, 1282},
        {16008, 1282},
        {16282, 1311},
        {16282, 1311},
        {16534, 1311},
        {16534, 1311},
        {16777, 1303},
        {16777, 1303},
        {17029, 1298},
        {17029, 1298},
        {17285, 1279},
        {17285, 1279},
        {17538, 1258},
        {17538, 1258},
        {17798, 1264},
        {17798, 1264},
        {18078, 1300},
        {18078, 1300},
        {18360, 1330},
        {18360, 1330},
        {18635, 1349},
        {18635, 1349},
        {18923, 1382},
        {18923, 1382},
        {19229, 1429},
        {19229, 1429},
        {19520, 1442},
        {19520, 1442},
        {19779, 1421},
        {19779, 1421},
        {20027, 1394},
        {20027, 1394},
        {20253, 1334},
        {20253, 1334},
        {20447, 1222},
        {20447, 1222},
        {20608, 1094},
        {20608, 1094},
        {20739, 965},
        {20739, 965},
        {20739, 965},
        {20834, 811},
        {20834, 811},
        {20903, 656},
        {20903, 656}
    };

    public static double []wristPoints = new double[]{
        0.334662586,
        0.334662586,
        0.334662586,
        0.334662586,
        0.334662586,
        0.334662586,
        0.334662586,
        0.334662586,
        0.334662586,
        0.334662586,
        0.334662586,
        0.334662586,
        0.334662586,
        0.334662586,
        0.333780736,
        0.333780736,
        0.333780736,
        0.333780736,
        0.333780736,
        0.333780736,
        0.333780736,
        0.333780736,
        0.333780736,
        0.333780736,
        0.333780736,
        0.333780736,
        0.333780736,
        0.333780736,
        0.333780736,
        0.333780736,
        0.333780736,
        0.333780736,
        0.333780736,
        0.333780736,
        0.333780736,
        0.330780655,
        0.330780655,
        0.330780655,
        0.330780655,
        0.330780655,
        0.330780655,
        0.330780655,
        0.330780655,
        0.330780655,
        0.330780655,
        0.330780655,
        0.330780655,
        0.330780655,
        0.330780655,
        0.330780655,
        0.330780655,
        0.330780655,
        0.330780655,
        0.330780655,
        0.330780655,
        0.316101521,
        0.316101521,
        0.316101521,
        0.316101521,
        0.316101521,
        0.316101521,
        0.316101521,
        0.316101521,
        0.316101521,
        0.316101521,
        0.316101521,
        0.316101521,
        0.316101521,
        0.316101521,
        0.316101521,
        0.316101521,
        0.316101521,
        0.316101521,
        0.316101521,
        0.316101521,
        0.301422447,
        0.301422447,
        0.301422447,
        0.301422447,
        0.301422447,
        0.301422447,
        0.301422447,
        0.301422447,
        0.301422447,
        0.301422447,
        0.301422447,
        0.301422447,
        0.301422447,
        0.301422447,
        0.301422447
    };

};

