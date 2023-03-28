package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;

import frc.robot.state.arm.ArmStateMachine;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve;

public class _5_Charger_Mid_2pc_Blue extends PathPlannerCommandGroup {
    private static String pathPlannerFile = "_5_Charger_Mid_2pc_Blue";

    public _5_Charger_Mid_2pc_Blue( Swerve s_Swerve, PoseEstimatorSubsystem s_PoseEstimatorSubsystem, ArmStateMachine sm_ArmStateMachine) {
        super(pathPlannerFile,  s_Swerve, s_PoseEstimatorSubsystem, sm_ArmStateMachine,new PathConstraints(4, 2.0));
    }
}
