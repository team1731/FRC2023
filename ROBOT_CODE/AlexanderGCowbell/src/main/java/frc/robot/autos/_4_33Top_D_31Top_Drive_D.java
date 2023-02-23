package frc.robot.autos;

import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve;

public class _4_33Top_D_31Top_Drive_D extends PathWeaverAutoCommandGroup {
    private static String[] trajectoryPaths = {"paths/A4P1.wpilib.json", 
                                               "paths/A4P2.wpilib.json",
                                               "paths/A4P3.wpilib.json"
                                            };

    public _4_33Top_D_31Top_Drive_D(boolean isRedAlliance, Swerve s_Swerve, PoseEstimatorSubsystem s_PoseEstimatorSubsystem){
        super(isRedAlliance, trajectoryPaths, s_Swerve, s_PoseEstimatorSubsystem);
    }
}
