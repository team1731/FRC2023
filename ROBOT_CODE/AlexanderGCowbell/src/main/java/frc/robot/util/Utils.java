package frc.robot.util;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DataLogManager;

public class Utils {
	public static void printTrajectory(String name, Trajectory trajectory) {
		DataLogManager.log("\n" + name + ":");
		double duration = trajectory.getTotalTimeSeconds();
		DataLogManager.log("trajectory duration " + duration);
		for (int i = 0; i <= (int) duration * 2; i++) {
			Trajectory.State state = trajectory.sample(i / 2.0);
			System.out
					.println("state " + i + "                 poseMetersX " + state.poseMeters.getTranslation().getX());
			System.out
					.println("state " + i + "                 poseMetersY " + state.poseMeters.getTranslation().getY());
			DataLogManager.log(
					"state " + i + "         poseMetersTheta Deg " + state.poseMeters.getRotation().getDegrees());
			DataLogManager.log("state " + i + "     velocityMetersPerSecond " + state.velocityMetersPerSecond);
		}
		Trajectory.State state = trajectory.sample(duration);
		DataLogManager.log("state (end)             poseMetersX " + state.poseMeters.getTranslation().getX());
		DataLogManager.log("state (end)             poseMetersY " + state.poseMeters.getTranslation().getY());
		DataLogManager.log("state (end)     poseMetersTheta Deg " + state.poseMeters.getRotation().getDegrees());
		DataLogManager.log("state (end) velocityMetersPerSecond " + state.velocityMetersPerSecond);
	}

	public static int Clamp(int val, int min, int max) {
		return Math.max(min, Math.min(max, val));
	}

	public static double Clamp(double val, double min, double max) {
		return Math.max(min, Math.min(max, val));
	}
}