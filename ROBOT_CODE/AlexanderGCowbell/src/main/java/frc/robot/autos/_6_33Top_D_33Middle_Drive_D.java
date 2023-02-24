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

public class _6_33Top_D_33Middle_Drive_D extends PathWeaverAutoCommandGroup {
    private static String[] trajectoryPaths = {"paths/A6P1.wpilib.json", 
                                               "paths/A6P2.wpilib.json",
                                               "paths/A6P3.wpilib.json"
                                            };

    public _6_33Top_D_33Middle_Drive_D(boolean isRedAlliance, Swerve s_Swerve, PoseEstimatorSubsystem s_PoseEstimatorSubsystem, ArmStateMachine sm_ArmStateMachine){
        super(isRedAlliance, s_Swerve, s_PoseEstimatorSubsystem, sm_ArmStateMachine);

        // TODO implement
    }
}
