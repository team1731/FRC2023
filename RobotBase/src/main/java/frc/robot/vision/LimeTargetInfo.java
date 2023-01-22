package frc.robot.vision;

import frc.robot.Constants.VisionConstants;

public class LimeTargetInfo {

	private double _x = 1.0;
	private double _y;
	private double _z;
	private double _timestamp;

	private double _area;
	private double _boxLength;
	private double _boxWidth;
	private double _targetDistance;

	public static LimeTargetInfo empty = new LimeTargetInfo(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

	public LimeTargetInfo(double y, double z, double area, double hor, double vert, double timestamp) {
		this._y = y;
		this._z = z;
		this._timestamp = timestamp;

		this._area = area;
		this._boxLength = vert;
		this._boxWidth = hor;
		
		double angleToGoalDegrees = VisionConstants.kCameraPitchAngleDegrees + z;
		double angleToGoalRadians = Math.toRadians(angleToGoalDegrees); // angleToGoalDegrees * (Math.PI / 180.0);

		//calculate distance
		double distanceFromLimelightToGoalMeters = (VisionConstants.kGoalHeight - VisionConstants.kCameraLensHeightMeters)/Math.tan(angleToGoalRadians);
		this._targetDistance = distanceFromLimelightToGoalMeters;
	}

	/**
	 * Returns the X of the target in robot coordinates (Z in target coordinates).
	 * This is locked at 1.
	 * 
	 * @return
	 */
	public double getX() {
		return _x;
	}

	/**
	 * Returns the Y of the target in robot coordinates (X in target coordinates).
	 * 
	 * @return
	 */
	public double getY() {
		return _y;
	}

	/**
	 * Returns the Z of the target in robot coordinates (Y in target coordinates).
	 * 
	 * @return
	 */
	public double getZ() {
		return _z;
	}

	/**
	 * Returns the timestamp when this data was captured
	 * 
	 * @return
	 */
	public double getTimeCaptured() {
		return _timestamp;
	}

	/**
	 * Returns the area of the bounding box around the target in percentage of image
	 * (0-100%). Camera resolution is 320x240.
	 * 
	 * @return
	 */
	public double getArea() {
		return _area;
	}

	/**
	 * Returns the length of the bounding box around the target in pixels.
	 * 
	 * @return
	 */
	public double getLength() {
		return _boxLength;
	}

	/**
	 * Returns the width of the bounding box around the target in pixels.
	 * 
	 * @return
	 */
	public double getWidth() {
		return _boxWidth;
	}

		/**
	 * Returns the width of the bounding box around the target in pixels.
	 * 
	 * @return
	 */
	public double getTargetDistance() {
		return _targetDistance;
	}

}