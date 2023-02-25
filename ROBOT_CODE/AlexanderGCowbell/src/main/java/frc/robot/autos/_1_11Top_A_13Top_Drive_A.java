package frc.robot.autos;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.GamePiece;
import frc.robot.commands.AutoScoreCommand;
import frc.robot.commands.AutoPickupCommand;
import frc.robot.state.arm.ArmSequence;
import frc.robot.state.arm.ArmStateMachine;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve;

public class _1_11Top_A_13Top_Drive_A extends SequentialCommandGroup { //extends PathWeaverAutoCommandGroup {
    private static String[] trajectoryPaths = {"paths/A1P1.wpilib.json", 
                                               "paths/A1P2.wpilib.json",
                                               "paths/A1P3.wpilib.json"
                                              };

    public _1_11Top_A_13Top_Drive_A(boolean isRedAlliance, Swerve s_Swerve, PoseEstimatorSubsystem s_PoseEstimatorSubsystem, ArmStateMachine sm_ArmStateMachine) {
        // super(isRedAlliance, s_Swerve, s_PoseEstimatorSubsystem, sm_ArmStateMachine);

        // // Load the pathweaver trajectories
        // Trajectory[] trajectories = loadTrajectories(trajectoryPaths);

        // // Configure trajectories
        // Trajectory trajectory0 = configureTrajectory(trajectories[0], Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(0.0));
        // Trajectory trajectory1 = configureTrajectory(trajectories[1], Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(0.0));
        // Trajectory trajectory2 = configureTrajectory(trajectories[2], Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(0.0));

        // // Build and order commands
        // addCommands(new InstantCommand(() -> s_PoseEstimatorSubsystem.setCurrentPose(configuredTrajectories.get(0).getInitialPose())));

        // // Step 1: Start at station, score cone
        // addCommands(new AutoScoreCommand(sm_ArmStateMachine, ArmSequence.SCORE_HIGH, GamePiece.CONE));

        // // Step 2: Drive out and pickup cube
        // //addCommands(createSwerveCommand(trajectory0)); 
        // addCommands(new AutoPickupCommand(sm_ArmStateMachine, ArmSequence.PICKUP_LOW, GamePiece.CUBE));

        // // Step 3: Drive back to station, score cube
        // //addCommands(createSwerveCommand(trajectory1));
        // addCommands(new AutoScoreCommand(sm_ArmStateMachine, ArmSequence.SCORE_HIGH, GamePiece.CUBE));

        // // Step 4: Drive out and pickup another cube
        // //addCommands(createSwerveCommand(trajectory2));
        // addCommands(new AutoPickupCommand(sm_ArmStateMachine, ArmSequence.PICKUP_LOW, GamePiece.CUBE));
 
 
 
    // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    PathConstraints pathConstraints = new PathConstraints(0.25, 0.25); //Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("test", pathConstraints); // "A1"

    // This is just an example event map. It would be better to have a constant, global event map
    // in your code that will be used by all path following commands.
    HashMap<String, Command> eventMap = new HashMap<>();
    //eventMap.put("ScoreCone", new PrintCommand("Passed marker 1"));
    eventMap.put("ScoreCone", new SequentialCommandGroup(new WaitCommand(3), new AutoScoreCommand(sm_ArmStateMachine, ArmSequence.SCORE_HIGH, GamePiece.CONE)));
    eventMap.put("IntakeCube", new PrintCommand("Intaking Cube"));
    eventMap.put("ScoreCube", new PrintCommand("Scoring Cube"));
    //eventMap.put("intakeDown", new IntakeDown());

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        s_PoseEstimatorSubsystem::getCurrentPose, // Pose2d supplier
        s_PoseEstimatorSubsystem::setCurrentPose, // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
        new PIDConstants(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(Constants.Swerve.angleKP, Constants.Swerve.angleKI, 0), //Constants.Swerve.angleKD), // PID constants to correct for rotation error (used to create the rotation controller)
        s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        s_Swerve,
        s_PoseEstimatorSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
    );

    Command fullAuto = autoBuilder.fullAuto(pathGroup);
    addCommands(fullAuto);
 
 
    }
}
