package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class PathWeaverAutoCommandGroup extends SequentialCommandGroup {
    
    void loadCoordinateSystem(boolean isRedAlliance){
        if(isRedAlliance){
            // TDB: use translated April Tags coordinate system
        }
        else{
            // TBD: use standard coordinate system
        }
    }

    public PathWeaverAutoCommandGroup(boolean isRedAlliance, String[] trajectoryPaths, Swerve s_Swerve, PoseEstimatorSubsystem s_PoseEstimatorSubsystem){

        loadCoordinateSystem(isRedAlliance);

        Trajectory[] trajectories = new Trajectory[trajectoryPaths.length];

        for(int i=0; i<trajectoryPaths.length; i++){
            try {
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryPaths[i]);
                String tP = trajectoryPath.toAbsolutePath().toString();
                if(!tP.contains("output")){
                    trajectoryPath = new File(tP.replace("paths", "paths" + File.separator + "output")).toPath();
                }
                DataLogManager.log(trajectoryPath.toAbsolutePath().toString());
                trajectories[i] = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory: " + trajectoryPaths[i], ex.getStackTrace());
            }
        }

        TrajectoryConfig config =
            new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);

        List<Trajectory> configuredTrajectories = new ArrayList<Trajectory>();
        for(Trajectory trajectory : trajectories){
            Pose2d initialPose = trajectory.getInitialPose();
            List<State> states = trajectory.getStates();
            List<Translation2d> waypoints = new ArrayList<Translation2d>();
            for(int i=1; i<states.size()-1; i++){
                waypoints.add(states.get(i).poseMeters.getTranslation());
            }
            State finalState = states.get(states.size()-1);
            Pose2d finalPose = finalState.poseMeters;
            Trajectory configuredTrajectory =
                TrajectoryGenerator.generateTrajectory(
                    // Start at the origin facing the +X direction
                    initialPose,
                    // Pass through these interior waypoints
                    waypoints,
                    // End at the final pose
                    finalPose,
                    config);
            configuredTrajectories.add(configuredTrajectory);
        }

        var xPiDController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
        var yPiDController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        List<SwerveControllerCommand> swerveControllerCommands = new ArrayList<SwerveControllerCommand>();
        for(Trajectory configuredTrajectory : configuredTrajectories){
            swerveControllerCommands.add(
                new SwerveControllerCommand(
                    configuredTrajectory,
                    s_PoseEstimatorSubsystem::getCurrentPose,
                    Constants.Swerve.swerveKinematics,
                    xPiDController,
                    yPiDController,
                    thetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve));
        }

        addCommands(new InstantCommand(() -> s_PoseEstimatorSubsystem.setCurrentPose(configuredTrajectories.get(0).getInitialPose())));
        for(SwerveControllerCommand swerveControllerCommand : swerveControllerCommands){
            addCommands(swerveControllerCommand);
        };
    
    }
    public void log(){
        
    }
}
