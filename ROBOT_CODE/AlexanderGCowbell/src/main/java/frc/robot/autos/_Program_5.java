package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;

import frc.robot.state.arm.ArmStateMachine;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve;

public class _Program_5 extends PathPlannerCommandGroup {
    private static String pathPlannerFile = "Program_5";

    public _Program_5(boolean isRedAlliance, Swerve s_Swerve, PoseEstimatorSubsystem s_PoseEstimatorSubsystem, ArmStateMachine sm_ArmStateMachine) {
        super(pathPlannerFile, isRedAlliance, s_Swerve, s_PoseEstimatorSubsystem, sm_ArmStateMachine,new PathConstraints(4, 2.0));
    }
}
