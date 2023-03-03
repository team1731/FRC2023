package frc.robot.autos;

import frc.robot.state.arm.ArmStateMachine;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve;

public class _Program_2 extends PathPlannerCommandGroup {
    private static String pathPlannerFile = "Program_2";

    public _Program_2(boolean isRedAlliance, Swerve s_Swerve, PoseEstimatorSubsystem s_PoseEstimatorSubsystem, ArmStateMachine sm_ArmStateMachine) {
        super(pathPlannerFile, isRedAlliance, s_Swerve, s_PoseEstimatorSubsystem, sm_ArmStateMachine);
    }
}
