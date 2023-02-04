package frc.robot.autos;

import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve;

public class _3_31Top_C_Engage extends PathWeaverAutoCommandGroup {
    private static String[] trajectoryPaths = {"paths/A3P1.wpilib.json", 
                                               "paths/A3P2.wpilib.json"
                                              };

    public _3_31Top_C_Engage(boolean isRedAlliance, Swerve s_Swerve, PoseEstimatorSubsystem s_PoseEstimatorSubsystem){
        super(isRedAlliance, trajectoryPaths, s_Swerve, s_PoseEstimatorSubsystem);
    }
}
