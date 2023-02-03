package frc.robot.util;

public class AutoSwerveDebug {
	public double curTime;
	public double desiredX;
	public double desiredY;
	public double desiredTheta;
	public double actualX;
	public double actualY;
	public double actualTheta;
	public double gyroAngle;
	public double gyroXAccel;
	public double gyroYAccel;
	public double actualXAccel;
	public double actualYAccel;
	public double setXVelocity;
	public double setYVelocity;
	public double errorXvel;
	public double errorYvel;
	public double actualXVelocity;
	public double actualYVelocity;

	public AutoSwerveDebug(double curTime, double desiredX, double desiredY, double desiredTheta, double actualX,
			double actualY, double actualTheta, double gyroAngle, double gyroXAccel, double gyroYAccel,
			double actualXAccel, double actualYAccel, double setXVelocity, double setYVelocity, double errorXvel,
			double errorYvel, double actualXVelocity, double actualYVelocity) {
		this.curTime = curTime;
		this.desiredX = desiredX;
		this.desiredY = desiredY;
		this.desiredTheta = desiredTheta;
		this.actualX = actualX;
		this.actualY = actualY;
		this.actualTheta = actualTheta;
		this.gyroAngle = gyroAngle;
		this.gyroXAccel = gyroXAccel;
		this.gyroYAccel = gyroYAccel;
		this.actualXAccel = actualXAccel;
		this.actualYAccel = actualYAccel;
		this.setXVelocity = setXVelocity;
		this.setYVelocity = setYVelocity;
		this.errorXvel = errorXvel;
		this.errorYvel = errorYvel;
		this.actualYVelocity = actualYVelocity;
		this.actualYVelocity = actualYVelocity;
	}
}
