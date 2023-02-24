package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.state.arm.ArmStateMachine;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.util.log.MessageLog;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class PathWeaverAutoCommandGroup extends SequentialCommandGroup {

    protected boolean isRedAlliance = false;
    protected Swerve s_Swerve;
    protected PoseEstimatorSubsystem s_PoseEstimatorSubsystem;
    protected ArmStateMachine sm_ArmStateMachine;
    protected List<Trajectory> configuredTrajectories;
    private TrajectoryConfig trajectoryConfig;
    
    private void loadCoordinateSystem(boolean isRedAlliance){
        if(isRedAlliance){
            // TDB: use translated April Tags coordinate system
        }
        else{
            // TBD: use standard coordinate system
        }
    }

    public PathWeaverAutoCommandGroup(boolean isRedAlliance, Swerve s_Swerve, PoseEstimatorSubsystem s_PoseEstimatorSubsystem, ArmStateMachine sm_ArmStateMachine) {
        this.isRedAlliance = isRedAlliance;
        this.s_Swerve = s_Swerve;
        this.s_PoseEstimatorSubsystem = s_PoseEstimatorSubsystem;
        this.sm_ArmStateMachine = sm_ArmStateMachine;
        configuredTrajectories = new ArrayList<Trajectory>();

        loadCoordinateSystem(isRedAlliance);

        trajectoryConfig =
            new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);
    }

    protected Trajectory[] loadTrajectories(String[] trajectoryPaths) {
        Trajectory[] trajectories = new Trajectory[trajectoryPaths.length];

        for(int i=0; i<trajectoryPaths.length; i++){
            try {
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryPaths[i]);
                String tP = trajectoryPath.toAbsolutePath().toString();
                if(!tP.contains("output")){
                    trajectoryPath = new File(tP.replace("paths", "paths" + File.separator + "output")).toPath();
                }
                MessageLog.add(trajectoryPath.toAbsolutePath().toString());
                trajectories[i] = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory: " + trajectoryPaths[i], ex.getStackTrace());
            }
        }

        return trajectories;
    }

    protected Trajectory configureTrajectory(Trajectory trajectory, Rotation2d initialRotation, Rotation2d finalRotation) {
        List<State> states = trajectory.getStates();
        Pose2d initialPose = trajectory.getInitialPose();
        List<Translation2d> waypoints = new ArrayList<Translation2d>();
        for(int i=1; i<states.size()-1; i++){
            waypoints.add(states.get(i).poseMeters.getTranslation());
        }
        State finalState = states.get(states.size()-1);
        Pose2d finalPose = finalState.poseMeters;

        Trajectory configuredTrajectory = 
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(initialPose.getX(), initialPose.getY(), initialRotation),
                // Pass through these interior waypoints
                waypoints,
                // End at the final pose
                new Pose2d(finalPose.getX(), finalPose.getY(), finalRotation),
                trajectoryConfig);
        
        configuredTrajectories.add(configuredTrajectory);
        return configuredTrajectory;
    }

    protected SwerveControllerCommand createSwerveCommand(Trajectory trajectory) {
        var xPiDController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
        var yPiDController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);    

        return new SwerveControllerCommand(
            trajectory,
            s_PoseEstimatorSubsystem::getCurrentPose,
            Constants.Swerve.swerveKinematics,
            xPiDController,
            yPiDController,
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);
    }
}
