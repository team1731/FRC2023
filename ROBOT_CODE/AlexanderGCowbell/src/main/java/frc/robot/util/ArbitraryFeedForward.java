package frc.robot.util;

import frc.robot.Constants.ArmConstants;

public class ArbitraryFeedForward {
    public static double armExtension(double proximalTicks, double distalTicks) {
        //Calculates how far the arm is extended using trigonometry and angles derived from data from motors 
        double distalDistance = (ArmConstants.distalArmLength * java.lang.Math.cos(Math.toRadians(3)) * java.lang.Math.sin(Math.toRadians(distalTicks / ArmConstants.distalTicksPerDegree)));
        double proximalDistance = ArmConstants.proximalArmLength * java.lang.Math.sin(Math.toRadians(proximalTicks / ArmConstants.proximalTicksPerDegree));
        return distalDistance + proximalDistance; 
    }
    
    public static double distalArmExtension(double distalTicks){
        //Calculates how far the arm is extended using trigonometry and angles derived from data from motors 
        double distalDistance = (ArmConstants.distalArmLength * java.lang.Math.cos(Math.toRadians(3))) * java.lang.Math.sin(Math.toRadians(distalTicks / ArmConstants.distalTicksPerDegree));
        return distalDistance; 
    }

    public static double getArbitraryFeedForwardForProximalArm(double proximalTicks, double distalTicks){
        return ArmConstants.ThrottleAtFullExtensionDistalAndProximal * (armExtension(proximalTicks, distalTicks) / ArmConstants.armFullExtensionDistance);
    }

    public static double getArbitraryFeedForwardForDistalArm(double distalTicks){
        return ArmConstants.ThrottleAtFullExtensionDistal * (distalArmExtension(distalTicks) / ArmConstants.distalArmLength);
    }
}
