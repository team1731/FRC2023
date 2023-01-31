package frc.robot.autos;

import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve;

public class _1_11Top_A_13Top_Drive_A extends PathWeaverAutoCommandGroup {
    private static String[][] trajectoryPaths = {{"paths/A1P1red.wpilib.json", 
                                                  "paths/A1P2red.wpilib.json",
                                                  "paths/A1P3red.wpilib.json"},
                                                 {"paths/A1P1blue.wpilib.json", 
                                                  "paths/A1P2blue.wpilib.json",
                                                  "paths/A1P3blue.wpilib.json"},
                                                 };

    public _1_11Top_A_13Top_Drive_A(boolean isRedAlliance, Swerve s_Swerve, PoseEstimatorSubsystem s_PoseEstimatorSubsystem){
        super(isRedAlliance, trajectoryPaths, s_Swerve, s_PoseEstimatorSubsystem);
    }
}
