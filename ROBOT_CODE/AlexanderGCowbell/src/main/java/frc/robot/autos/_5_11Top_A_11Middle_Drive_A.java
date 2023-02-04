package frc.robot.autos;

import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve;

public class _5_11Top_A_11Middle_Drive_A extends PathWeaverAutoCommandGroup {
    private static String[] trajectoryPaths = {"paths/A5P1.wpilib.json", 
                                               "paths/A5P2.wpilib.json",
                                               "paths/A5P3.wpilib.json"
                                            };

    public _5_11Top_A_11Middle_Drive_A(boolean isRedAlliance, Swerve s_Swerve, PoseEstimatorSubsystem s_PoseEstimatorSubsystem){
        super(isRedAlliance, trajectoryPaths, s_Swerve, s_PoseEstimatorSubsystem);
    }
}
