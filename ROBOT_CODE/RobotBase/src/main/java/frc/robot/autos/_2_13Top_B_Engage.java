package frc.robot.autos;

import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve;

public class _2_13Top_B_Engage extends PathWeaverAutoCommandGroup {
    private static String[] trajectoryPaths = {"paths/A2P1.wpilib.json", 
                                               "paths/A2P2.wpilib.json"
                                              };

    public _2_13Top_B_Engage(boolean isRedAlliance, Swerve s_Swerve, PoseEstimatorSubsystem s_PoseEstimatorSubsystem){
        super(isRedAlliance, trajectoryPaths, s_Swerve, s_PoseEstimatorSubsystem);
    }
}
