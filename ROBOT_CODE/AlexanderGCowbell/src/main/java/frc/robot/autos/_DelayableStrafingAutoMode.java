package frc.robot.autos;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.Utils;

@Deprecated
public class _DelayableStrafingAutoMode {
	private int initialDelaySeconds;
	private int secondaryDelaySeconds;
	Command command;

	public Pose2d getInitialPose() {
		return null;
	}

	public double getAngleOffset() {
		return 0.0;
	}

	public Integer getFieldOrientation() {
		return null;
	}

	public _DelayableStrafingAutoMode(int initialDelaySeconds, int secondaryDelaySeconds) {
		this.initialDelaySeconds = initialDelaySeconds;
		this.secondaryDelaySeconds = secondaryDelaySeconds;
	}

	public _DelayableStrafingAutoMode() {
		this(0, 0);
	}

	public void setInitialDelaySeconds(int initialDelaySeconds) {
		this.initialDelaySeconds = initialDelaySeconds;
	}

	public void setSecondaryDelaySeconds(int secondaryDelaySeconds) {
		this.secondaryDelaySeconds = secondaryDelaySeconds;
	}

	public int getInitialDelaySeconds() {
		return initialDelaySeconds;
	}

	public int getSecondaryDelaySeconds() {
		return secondaryDelaySeconds;
	}

	public Command getCommand() {
		return command;
	}

	/**
	 * @param oldStates            -- original list of states from the (calculated)
	 *                             trajectory
	 * @param finalRotationDegrees -- desired final pose rotation -- to assign to
	 *                             last state in list
	 * @return -- list of fixed-up (unrotated) states (except for the last one in
	 *         the list)
	 */
	List<Trajectory.State> unrotateTrajectory(List<Trajectory.State> oldStates, double finalRotationDegrees) {
		List<Trajectory.State> newStates = new ArrayList<Trajectory.State>();
		int i = 0;
		for (Trajectory.State state : oldStates) {
			// instead of rotating the pose by its inverse (dumb)...
			// Rotation2d newRot = state.poseMeters.getRotation().rotateBy(new
			// Rotation2d(-state.poseMeters.getRotation().getRadians()));
			// simply assign a new Rotation having 0 degrees...
			double rotationDegrees = i++ == oldStates.size() - 1 ? finalRotationDegrees : 0;
			Pose2d newPose = new Pose2d(state.poseMeters.getTranslation(),
					new Rotation2d(Math.toRadians(rotationDegrees)));
			newStates.add(new Trajectory.State(state.timeSeconds, state.velocityMetersPerSecond,
					state.accelerationMetersPerSecondSq, newPose, state.curvatureRadPerMeter));
		}
		return newStates;
	}

	/**
	 * @param oldStates       -- original list of states from the (calculated)
	 *                        trajectory
	 * @param rotationDegrees -- desired final pose rotation -- to assign to last
	 *                        state in list
	 * @return -- list of fixed-up (unrotated) states (except for the last one in
	 *         the list)
	 */
	List<Trajectory.State> maintainTrajectory(List<Trajectory.State> oldStates, double rotationDegrees) {
		List<Trajectory.State> newStates = new ArrayList<Trajectory.State>();
		for (Trajectory.State state : oldStates) {
			// simply assign a new Rotation having rotationDegrees degrees...
			Pose2d newPose = new Pose2d(state.poseMeters.getTranslation(),
					new Rotation2d(Math.toRadians(rotationDegrees)));
			newStates.add(new Trajectory.State(state.timeSeconds, state.velocityMetersPerSecond,
					state.accelerationMetersPerSecondSq, newPose, state.curvatureRadPerMeter));
		}
		return newStates;
	}

	/**
	 * @param oldStates       -- original list of states from the (calculated)
	 *                        trajectory
	 * @param rotationDegrees -- desired final pose rotation -- to assign to last
	 *                        state in list
	 * @return -- list of fixed-up (unrotated) states (except for the last one in
	 *         the list)
	 */
	List<Trajectory.State> convertTrajectory(List<Trajectory.State> oldStates, double rotationDegrees) {
		List<Trajectory.State> newStates = new ArrayList<Trajectory.State>();
		for (Trajectory.State state : oldStates) {
			// simply assign a new Rotation having rotationDegrees degrees...
			double x_meters = state.poseMeters.getX() / 39.37;
			double y_meters = state.poseMeters.getY() / 39.37;
			Pose2d newPose = new Pose2d(new Translation2d(x_meters, y_meters),
					new Rotation2d(Math.toRadians(rotationDegrees)));
			newStates.add(new Trajectory.State(state.timeSeconds, state.velocityMetersPerSecond,
					state.accelerationMetersPerSecondSq, newPose, state.curvatureRadPerMeter));
		}
		return newStates;
	}

	enum TrajectoryDirection {
		FWD, REV
	}

	public TrajectoryConfig createTrajectoryConfig(TrajectoryDirection dir) {
		switch (dir) {
			case FWD:
				return new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
						AutoConstants.kMaxAccelerationMetersPerSecondSquared)
								// Add kinematics to ensure max speed is actually obeyed
								.setKinematics(DriveConstants.kDriveKinematics).setReversed(false);
			case REV:
				return new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
						AutoConstants.kMaxAccelerationMetersPerSecondSquared)
								// Add kinematics to ensure max speed is actually obeyed
								.setKinematics(DriveConstants.kDriveKinematics).setReversed(true);
		}
		return null;
	}

	enum TrajectoryHeading {
		UNROTATE, MAINTAIN, CONVERT_TO_METERS, DO_NOTHING
	}

	public Trajectory createTrajectory(String name, TrajectoryDirection dir, TrajectoryHeading mode, double value,
			double[][] points) {
		// waypoints
		List<Translation2d> waypoints = new ArrayList<Translation2d>();
		if (points.length > 2) {
			for (int i = 1; i < points.length - 1; i++) {
				waypoints.add(new Translation2d(points[i][0], points[i][1]));
			}
		}

		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
				// initial pose
				new Pose2d(points[0][0], points[0][1], new Rotation2d(Math.toRadians(points[0][2]))), waypoints,
				// ending pose
				new Pose2d(points[points.length - 1][0], points[points.length - 1][1],
						new Rotation2d(Math.toRadians(points[points.length - 1][2]))),
				createTrajectoryConfig(dir));

		switch (mode) {
			case UNROTATE:
				// undoes the wpilib pose calculation of heading so we can do a proper strafe
				// (supposes an initial heading of 0 deg)
				trajectory = new Trajectory(unrotateTrajectory(trajectory.getStates(), value));
				break;
			case MAINTAIN:
				// fixes the heading to a value throughout, so we can start the auto "at an
				// angle"
				// NOTE: if the angle is 0, it should accomplish the same thing as
				// unrotateTrajectory
				trajectory = new Trajectory(maintainTrajectory(trajectory.getStates(), value));
				break;
			case CONVERT_TO_METERS:
				// NOTE: conversion is done before this point, so just maintain trajectory at
				// this point
				// trajectory = new Trajectory(maintainTrajectory(trajectory.getStates(),
				// value)); break;
			case DO_NOTHING: // do not alter trajectory
		}
		Utils.printTrajectory(this.getClass().getSimpleName() + ": " + name, trajectory);
		return trajectory;
	}

	public double[][] convertPoints(double[][] originalPoints) {
		double[][] convertedPoints = new double[originalPoints.length][];
		for (int i = 0; i < originalPoints.length; i++) {
			double[] poseToConvert = new double[originalPoints[i].length];
			poseToConvert[0] = originalPoints[i][0] / 39.37;
			poseToConvert[1] = originalPoints[i][1] / 39.37;
			if (originalPoints[i].length == 3) {
				poseToConvert[2] = originalPoints[i][2];
			}
			convertedPoints[i] = poseToConvert;
		}
		return convertedPoints;
	}

	/*
	TODO DriveSubsystem is no longer in use, need to re-val this method, note: removed the _InstrumentedSwerveControllerCommand temporarily
	public Command createSwerveCommand(DriveSubsystem m_robotDrive, String name, double endingHeading,
			Trajectory trajectory, boolean trackTarget) {
				return null;
		// return new _InstrumentedSwerveControllerCommand(m_robotDrive, m_robotDrive.getCSVWriter(), trajectory, // trajectory
		// 																										// was
		// 																										// created
		// 																										// by
		// 																										// PathWeaver
		// 																										// and
		// 																										// read-in
		// 																										// from
		// 																										// a
		// 																										// file
		// 		endingHeading, // pass this value through
		// 		m_robotDrive::getPose, // Functional interface to feed supplier
		// 		DriveConstants.kDriveKinematics,

		// 		// Position controllers
		// 		new PIDController(AutoConstants.kPXController, 0, 0),
		// 		new PIDController(AutoConstants.kPYController, 0, 0),
		// 		new PIDController(AutoConstants.kPThetaController, 0, 0), trackTarget,

		// 		m_robotDrive::setModuleStates, m_robotDrive) {
		// 	@Override
		// 	public void end(boolean interrupted) {
		// 		super.end(interrupted);
		// 		DataLogManager.log("at end of swerve command, interrupted=" + interrupted);
		// 	}
		// };
	}
	*/

	/*
	 * TODO DriveSubsystem is no longer in use, need to re-val this method, note: removed the _InstrumentedSwerveControllerCommand temporarily
	public _InstrumentedSwerveControllerCommand createSwerveCommand(DriveSubsystem m_robotDrive, String name,
			TrajectoryDirection dir, TrajectoryHeading mode, double value, double[][] points, boolean trackTarget) {
		if (mode == TrajectoryHeading.CONVERT_TO_METERS) {
			points = convertPoints(points);
		}
		return null;
		// return new _InstrumentedSwerveControllerCommand(m_robotDrive, m_robotDrive.getCSVWriter(),
		// 		createTrajectory(name, dir, mode, value, points), m_robotDrive::getPose, // Functional interface to feed
		// 																					// supplier
		// 		DriveConstants.kDriveKinematics,

		// 		// Position controllers
		// 		new PIDController(AutoConstants.kPXController, 0, 0),
		// 		new PIDController(AutoConstants.kPYController, 0, 0),
		// 		new PIDController(AutoConstants.kPThetaController, 0, 0), trackTarget,

		// 		m_robotDrive::setModuleStates, m_robotDrive) {
		// 	@Override
		// 	public void end(boolean interrupted) {
		// 		super.end(interrupted);
		// 		DataLogManager.log("at end of swerve command, interrupted=" + interrupted);
		// 	}
		// };
	}
	*/

}
