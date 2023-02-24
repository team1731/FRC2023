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

public class _5_11Top_A_11Middle_Drive_A extends PathWeaverAutoCommandGroup {
    private static String[] trajectoryPaths = {"paths/A5P1.wpilib.json", 
                                               "paths/A5P2.wpilib.json",
                                               "paths/A5P3.wpilib.json"
                                            };

    public _5_11Top_A_11Middle_Drive_A(boolean isRedAlliance, Swerve s_Swerve, PoseEstimatorSubsystem s_PoseEstimatorSubsystem, ArmStateMachine sm_ArmStateMachine){
        super(isRedAlliance, s_Swerve, s_PoseEstimatorSubsystem, sm_ArmStateMachine);

        // TODO implement
    }
}
