package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.subsystems.PoseEstimatorSubsystem;


public class _9_Move_Forward extends SequentialCommandGroup {
	public _9_Move_Forward(Swerve s_Swerve, PoseEstimatorSubsystem s_PoseEstimatorSubsystem) {
		TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

		// Trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Initial point
                new Pose2d(0, 0, new Rotation2d(0)),
                // Waypoint
                List.of(new Translation2d(0.5, 0.0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(1.0, 0.0, new Rotation2d(0)),
                config);
		
		var thetaController =
				new ProfiledPIDController(
					Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		SwerveControllerCommand swerveControllerCommand =
			new SwerveControllerCommand(
				exampleTrajectory,
				s_PoseEstimatorSubsystem::getCurrentPose,
				Constants.Swerve.swerveKinematics,
				new PIDController(Constants.AutoConstants.kPXController, 0, 0),
				new PIDController(Constants.AutoConstants.kPYController, 0, 0),
				thetaController,
				s_Swerve::setModuleStates,
				s_Swerve);

		addCommands(
            new InstantCommand(() -> s_PoseEstimatorSubsystem.setCurrentPose(exampleTrajectory.getInitialPose())),
            swerveControllerCommand
        );
	}
}
