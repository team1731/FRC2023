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

public class _2_13Top_B_Engage extends PathWeaverAutoCommandGroup {
    private static String[] trajectoryPaths = {"paths/A2P1.wpilib.json", 
                                               "paths/A2P2.wpilib.json"
                                              };

    public _2_13Top_B_Engage(boolean isRedAlliance, Swerve s_Swerve, PoseEstimatorSubsystem s_PoseEstimatorSubsystem, ArmStateMachine sm_ArmStateMachine){
        super(isRedAlliance, s_Swerve, s_PoseEstimatorSubsystem, sm_ArmStateMachine);

        // Load the pathweaver trajectories
        Trajectory[] trajectories = loadTrajectories(trajectoryPaths);

        // Configure trajectories
        Trajectory trajectory0 = configureTrajectory(trajectories[0], Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(0.0));
        Trajectory trajectory1 = configureTrajectory(trajectories[1], Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(0.0));

        // Build and order commands
        addCommands(new InstantCommand(() -> s_PoseEstimatorSubsystem.setCurrentPose(configuredTrajectories.get(0).getInitialPose())));

        // Step 1: Start at station, score cone
        addCommands(new AutoScoreCommand(sm_ArmStateMachine, ArmSequence.SCORE_HIGH, GamePiece.CONE));

        // Step 2: Drive out and pickup cube
        addCommands(createSwerveCommand(trajectory0)); 
        addCommands(new AutoPickupCommand(sm_ArmStateMachine, ArmSequence.PICKUP_LOW, GamePiece.CUBE));

        // Step 3: Drive to charging station
        addCommands(createSwerveCommand(trajectory1));
    }
}
