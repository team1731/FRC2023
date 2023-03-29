package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;

import frc.robot.Constants.AutoConstants;
import frc.robot.state.arm.ArmStateMachine;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve;

public class _1_Charger_Mid_1pc_Blue extends PathPlannerCommandGroup {
    private static String pathPlannerFile = "_1_Charger_Mid_1pc_Blue";


    public _1_Charger_Mid_1pc_Blue( Swerve s_Swerve, PoseEstimatorSubsystem s_PoseEstimatorSubsystem, ArmStateMachine sm_ArmStateMachine) {
        super(pathPlannerFile,  s_Swerve, s_PoseEstimatorSubsystem, sm_ArmStateMachine,new PathConstraints(3.0, 1.5));
    }
}
