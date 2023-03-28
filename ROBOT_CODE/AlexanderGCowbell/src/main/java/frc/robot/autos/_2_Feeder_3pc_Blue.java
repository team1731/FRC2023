package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;

import frc.robot.state.arm.ArmStateMachine;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve;

public class _2_Feeder_3pc_Blue extends PathPlannerCommandGroup {
    private static String pathPlannerFile = "_2_Feeder_3pc_Blue";

    public _2_Feeder_3pc_Blue( Swerve s_Swerve, PoseEstimatorSubsystem s_PoseEstimatorSubsystem, ArmStateMachine sm_ArmStateMachine) {
        super(pathPlannerFile,  s_Swerve, s_PoseEstimatorSubsystem, sm_ArmStateMachine,new PathConstraints(3, 1.8));
    }
}
