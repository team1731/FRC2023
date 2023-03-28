package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;

import frc.robot.state.arm.ArmStateMachine;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve;

public class _4_Feeder_2pc_Charger_Blue extends PathPlannerCommandGroup {
    private static String pathPlannerFile = "_4_Feeder_2pc_Charger_Blue";

    public _4_Feeder_2pc_Charger_Blue( Swerve s_Swerve, PoseEstimatorSubsystem s_PoseEstimatorSubsystem, ArmStateMachine sm_ArmStateMachine) {
        super(pathPlannerFile, s_Swerve, s_PoseEstimatorSubsystem, sm_ArmStateMachine,new PathConstraints(4, 2.0));
    }
}
