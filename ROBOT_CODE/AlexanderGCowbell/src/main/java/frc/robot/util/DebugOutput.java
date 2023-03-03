package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class DebugOutput {
	public double t;
	public double pose_x;
	public double pose_y;
	public double pose_theta;
	public double headingRadians;
	public double raw_gyro;
	public double poseErrorX;
	public double poseErrorY;
	public double poseErrorTheta;
	// public double linear_displacement;
	// public double linear_velocity;
	// public double profile_displacement;
	// public double profile_velocity;
	// public double velocity_command_dx;
	// public double velocity_command_dy;
	// public double velocity_command_dtheta;
	// public double steering_command_dx;
	// public double steering_command_dy;
	// public double steering_command_dtheta;
	// public double cross_track_error;
	// public double along_track_error;
	// public double lookahead_point_x;
	// public double lookahead_point_y;
	// public double lookahead_point_velocity;

	public void update(double t, SwerveDriveOdometry m_odometry, double headingRadians, double raw_gyro) {
		this.t = t;
		pose_x = m_odometry.getPoseMeters().getTranslation().getX();
		pose_y = m_odometry.getPoseMeters().getTranslation().getY();
		pose_theta = m_odometry.getPoseMeters().getRotation().getDegrees();
		this.headingRadians = headingRadians;
		this.raw_gyro = raw_gyro;
	}

	public void update(double t, Pose2d poseError, ChassisSpeeds targetChassisSpeeds,
			SwerveModuleState[] targetModuleStates) {
		this.t = t;
		this.poseErrorX = poseError.getTranslation().getX();
		this.poseErrorY = poseError.getTranslation().getY();
		this.poseErrorTheta = poseError.getRotation().getDegrees();
	}

}
