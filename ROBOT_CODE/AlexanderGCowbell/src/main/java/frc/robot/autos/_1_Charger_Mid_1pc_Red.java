package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;

import frc.robot.Constants.AutoConstants;
import frc.robot.state.arm.ArmStateMachine;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve;

public class _1_Charger_Mid_1pc_Red extends PathPlannerCommandGroup {
    private static String pathPlannerFile = "_1_Charger_Mid_1pc_Red";


    public _1_Charger_Mid_1pc_Red( Swerve s_Swerve, PoseEstimatorSubsystem s_PoseEstimatorSubsystem, ArmStateMachine sm_ArmStateMachine) {
        super(pathPlannerFile,  s_Swerve, s_PoseEstimatorSubsystem, sm_ArmStateMachine,new PathConstraints(2.5, 1.0));
    }
}
