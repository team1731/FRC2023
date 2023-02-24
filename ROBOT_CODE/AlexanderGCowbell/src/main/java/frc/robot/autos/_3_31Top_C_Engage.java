package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.GamePiece;
import frc.robot.commands.AutoScoreCommand;
import frc.robot.commands.AutoPickupCommand;
import frc.robot.state.arm.ArmSequence;
import frc.robot.state.arm.ArmStateMachine;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve;

public class _3_31Top_C_Engage extends PathWeaverAutoCommandGroup {
    private static String[] trajectoryPaths = {"paths/A3P1.wpilib.json", 
                                               "paths/A3P2.wpilib.json"
                                              };

    public _3_31Top_C_Engage(boolean isRedAlliance, Swerve s_Swerve, PoseEstimatorSubsystem s_PoseEstimatorSubsystem, ArmStateMachine sm_ArmStateMachine){
        super(isRedAlliance, s_Swerve, s_PoseEstimatorSubsystem, sm_ArmStateMachine);

        // TODO implement
    }
}
