package frc.robot.autos;

import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve;

public class _6_33Top_D_33Middle_Drive_D extends PathWeaverAutoCommandGroup {
    private static String[] trajectoryPaths = {"paths/A6P1.wpilib.json", 
                                               "paths/A6P2.wpilib.json",
                                               "paths/A6P3.wpilib.json"
                                            };

    public _6_33Top_D_33Middle_Drive_D(boolean isRedAlliance, Swerve s_Swerve, PoseEstimatorSubsystem s_PoseEstimatorSubsystem){
        super(isRedAlliance, trajectoryPaths, s_Swerve, s_PoseEstimatorSubsystem);
    }
}
