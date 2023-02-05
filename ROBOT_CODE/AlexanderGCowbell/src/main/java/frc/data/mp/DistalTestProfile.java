package frc.data.mp;
	
public class DistalTestProfile {	
    public static MotionProfile getProfile() {
        return new MotionProfile(kNumPoints, Points, true);
    }
    		
	public static final int kNumPoints = 189;	

	// Position (rotations)	Velocity (RPM)	Duration (ms)
	public static double [][]Points = new double[][]{		
        {0,	0	,10},
        {0.00019047619047619,	2.285714286	,10},
        {0.000857142857142857,	5.714285714	,10},
        {0.00219047619047619,	10.28571429	,10},
        {0.00438095238095238,	16	,10},
        {0.00761904761904762,	22.85714286	,10},
        {0.0120952380952381,	30.85714286	,10},
        {0.018,	40	,10},
        {0.0255238095238095,	50.28571429	,10},
        {0.0348571428571429,	61.71428571	,10},
        {0.0461904761904762,	74.28571429	,10},
        {0.0597142857142857,	88	,10},
        {0.0756190476190476,	102.8571429	,10},
        {0.0940952380952381,	118.8571429	,10},
        {0.115333333333333,	136	,10},
        {0.13952380952381,	154.2857143	,10},
        {0.166857142857143,	173.7142857	,10},
        {0.19752380952381,	194.2857143	,10},
        {0.231714285714286,	216	,10},
        {0.269619047619048,	238.8571429	,10},
        {0.311428571428571,	262.8571429	,10},
        {0.357238095238095,	286.8571429	,10},
        {0.407047619047619,	310.8571429	,10},
        {0.460857142857143,	334.8571429	,10},
        {0.518666666666667,	358.8571429	,10},
        {0.580476190476191,	382.8571429	,10},
        {0.646285714285714,	406.8571429	,10},
        {0.716095238095238,	430.8571429	,10},
        {0.789904761904762,	454.8571429	,10},
        {0.867714285714286,	478.8571429	,10},
        {0.94952380952381,	502.8571429	,10},
        {1.03533333333333,	526.8571429	,10},
        {1.12514285714286,	550.8571429	,10},
        {1.21895238095238,	574.8571429	,10},
        {1.3167619047619,	598.8571429	,10},
        {1.41857142857143,	622.8571429	,10},
        {1.52438095238095,	646.8571429	,10},
        {1.63419047619048,	670.8571429	,10},
        {1.748,	694.8571429	,10},
        {1.86580952380952,	718.8571429	,10},
        {1.98761904761905,	742.8571429	,10},
        {2.1132380952381,	764.5714286	,10},
        {2.24238095238095,	785.1428571	,10},
        {2.37485714285714,	804.5714286	,10},
        {2.51047619047619,	822.8571429	,10},
        {2.64904761904762,	840	,10},
        {2.79038095238095,	856	,10},
        {2.93428571428571,	870.8571429	,10},
        {3.08057142857143,	884.5714286	,10},
        {3.22904761904762,	897.1428571	,10},
        {3.37952380952381,	908.5714286	,10},
        {3.53180952380952,	918.8571429	,10},
        {3.68571428571429,	928	,10},
        {3.84104761904762,	936	,10},
        {3.99761904761905,	942.8571429	,10},
        {4.1552380952381,	948.5714286	,10},
        {4.31371428571429,	953.1428571	,10},
        {4.47285714285714,	956.5714286	,10},
        {4.63247619047619,	958.8571429	,10},
        {4.79238095238095,	960	,10},
        {4.95238095238095,	960	,10},
        {5.11238095238095,	960	,10},
        {5.27238095238095,	960	,10},
        {5.43238095238095,	960	,10},
        {5.59238095238095,	960	,10},
        {5.75238095238095,	960	,10},
        {5.91238095238095,	960	,10},
        {6.07238095238095,	960	,10},
        {6.23238095238095,	960	,10},
        {6.39238095238095,	960	,10},
        {6.55238095238095,	960	,10},
        {6.71238095238095,	960	,10},
        {6.87238095238095,	960	,10},
        {7.03238095238095,	960	,10},
        {7.19238095238095,	960	,10},
        {7.35238095238095,	960	,10},
        {7.51238095238095,	960	,10},
        {7.67238095238095,	960	,10},
        {7.83238095238096,	960	,10},
        {7.99238095238096,	960	,10},
        {8.15238095238096,	960	,10},
        {8.31238095238096,	960	,10},
        {8.47238095238096,	960	,10},
        {8.63238095238096,	960	,10},
        {8.79238095238096,	960	,10},
        {8.95238095238096,	960	,10},
        {9.11238095238096,	960	,10},
        {9.27238095238096,	960	,10},
        {9.43238095238096,	960	,10},
        {9.59238095238096,	960	,10},
        {9.75238095238096,	960	,10},
        {9.91238095238096,	960	,10},
        {10.072380952381,	960	,10},
        {10.232380952381,	960	,10},
        {10.392380952381,	960	,10},
        {10.552380952381,	960	,10},
        {10.712380952381,	960	,10},
        {10.872380952381,	960	,10},
        {11.032380952381,	960	,10},
        {11.192380952381,	960	,10},
        {11.352380952381,	960	,10},
        {11.512380952381,	960	,10},
        {11.672380952381,	960	,10},
        {11.832380952381,	960	,10},
        {11.992380952381,	960	,10},
        {12.152380952381,	960	,10},
        {12.312380952381,	960	,10},
        {12.472380952381,	960	,10},
        {12.632380952381,	960	,10},
        {12.792380952381,	960	,10},
        {12.952380952381,	960	,10},
        {13.112380952381,	960	,10},
        {13.272380952381,	960	,10},
        {13.432380952381,	960	,10},
        {13.592380952381,	960	,10},
        {13.752380952381,	960	,10},
        {13.912380952381,	960	,10},
        {14.072380952381,	960	,10},
        {14.232380952381,	960	,10},
        {14.392380952381,	960	,10},
        {14.552380952381,	960	,10},
        {14.712380952381,	960	,10},
        {14.872380952381,	960	,10},
        {15.032380952381,	960	,10},
        {15.192380952381,	960	,10},
        {15.352380952381,	960	,10},
        {15.512380952381,	960	,10},
        {15.672380952381,	960	,10},
        {15.832380952381,	960	,10},
        {15.992380952381,	960	,10},
        {16.1521904761905,	957.7142857	,10},
        {16.3115238095238,	954.2857143	,10},
        {16.4701904761905,	949.7142857	,10},
        {16.628,	944	,10},
        {16.7847619047619,	937.1428571	,10},
        {16.9402857142857,	929.1428571	,10},
        {17.094380952381,	920	,10},
        {17.2468571428571,	909.7142857	,10},
        {17.3975238095238,	898.2857143	,10},
        {17.5461904761905,	885.7142857	,10},
        {17.6926666666667,	872	,10},
        {17.8367619047619,	857.1428571	,10},
        {17.9782857142857,	841.1428571	,10},
        {18.1170476190476,	824	,10},
        {18.2528571428571,	805.7142857	,10},
        {18.3855238095238,	786.2857143	,10},
        {18.5148571428571,	765.7142857	,10},
        {18.6406666666667,	744	,10},
        {18.7627619047619,	721.1428571	,10},
        {18.8809523809524,	697.1428571	,10},
        {18.9951428571429,	673.1428571	,10},
        {19.1053333333333,	649.1428571	,10},
        {19.2115238095238,	625.1428571	,10},
        {19.3137142857143,	601.1428571	,10},
        {19.4119047619048,	577.1428571	,10},
        {19.5060952380952,	553.1428571	,10},
        {19.5962857142857,	529.1428571	,10},
        {19.6824761904762,	505.1428571	,10},
        {19.7646666666667,	481.1428571	,10},
        {19.8428571428571,	457.1428571	,10},
        {19.9170476190476,	433.1428571	,10},
        {19.9872380952381,	409.1428571	,10},
        {20.0534285714286,	385.1428571	,10},
        {20.115619047619,	361.1428571	,10},
        {20.1738095238095,	337.1428571	,10},
        {20.228,	313.1428571	,10},
        {20.2781904761905,	289.1428571	,10},
        {20.324380952381,	265.1428571	,10},
        {20.3665714285714,	241.1428571	,10},
        {20.4047619047619,	217.1428571	,10},
        {20.4391428571429,	195.4285714	,10},
        {20.47,	174.8571429	,10},
        {20.4975238095238,	155.4285714	,10},
        {20.5219047619048,	137.1428571	,10},
        {20.5433333333333,	120	,10},
        {20.562,	104	,10},
        {20.5780952380952,	89.14285714	,10},
        {20.5918095238095,	75.42857143	,10},
        {20.6033333333333,	62.85714286	,10},
        {20.6128571428571,	51.42857143	,10},
        {20.6205714285714,	41.14285714	,10},
        {20.6266666666667,	32	,10},
        {20.6313333333333,	24	,10},
        {20.6347619047619,	17.14285714	,10},
        {20.6371428571429,	11.42857143	,10},
        {20.6386666666667,	6.857142857	,10},
        {20.6395238095238,	3.428571429	,10},
        {20.6399047619048,	1.142857143	,10},
        {20.64,	0	,10}		
    };			
}			
