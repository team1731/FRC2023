package frc.robot.util;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.util.log.MessageLog;

public class Utils {
	public static void printTrajectory(String name, Trajectory trajectory) {
		MessageLog.add("\n" + name + ":");
		double duration = trajectory.getTotalTimeSeconds();
		MessageLog.add("trajectory duration " + duration);
		for (int i = 0; i <= (int) duration * 2; i++) {
			Trajectory.State state = trajectory.sample(i / 2.0);
			System.out
					.println("state " + i + "                 poseMetersX " + state.poseMeters.getTranslation().getX());
			System.out
					.println("state " + i + "                 poseMetersY " + state.poseMeters.getTranslation().getY());
			MessageLog.add(
					"state " + i + "         poseMetersTheta Deg " + state.poseMeters.getRotation().getDegrees());
			MessageLog.add("state " + i + "     velocityMetersPerSecond " + state.velocityMetersPerSecond);
		}
		Trajectory.State state = trajectory.sample(duration);
		MessageLog.add("state (end)             poseMetersX " + state.poseMeters.getTranslation().getX());
		MessageLog.add("state (end)             poseMetersY " + state.poseMeters.getTranslation().getY());
		MessageLog.add("state (end)     poseMetersTheta Deg " + state.poseMeters.getRotation().getDegrees());
		MessageLog.add("state (end) velocityMetersPerSecond " + state.velocityMetersPerSecond);
	}

	public static int Clamp(int val, int min, int max) {
		return Math.max(min, Math.min(max, val));
	}

	public static double Clamp(double val, double min, double max) {
		return Math.max(min, Math.min(max, val));
	}
}