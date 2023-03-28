package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;

import frc.robot.state.arm.ArmStateMachine;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve;

public class _2_Feeder_3pc_Red extends PathPlannerCommandGroup {
    private static String pathPlannerFile = "_2_Feeder_3pc_Red";

    public _2_Feeder_3pc_Red(Swerve s_Swerve, PoseEstimatorSubsystem s_PoseEstimatorSubsystem, ArmStateMachine sm_ArmStateMachine) {
        super(pathPlannerFile,  s_Swerve, s_PoseEstimatorSubsystem, sm_ArmStateMachine, new PathConstraints(4, 2.0));
    }
}
