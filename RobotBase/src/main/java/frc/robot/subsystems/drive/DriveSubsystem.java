/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ToggleableSubsystem;
import frc.robot.util.AutoSwerveDebug;
import frc.robot.util.ReflectingCSVWriter;
import frc.robot.util.SwerveModuleDebug;
//import frc.robot.util.Utils;;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends ToggleableSubsystem {

	@Override
	protected boolean getEnabled() {
		return true;
	}

	private final Timer m_timer = new Timer();

	private final ReflectingCSVWriter<AutoSwerveDebug> mCSVWriter1;
	private final ReflectingCSVWriter<SwerveModuleDebug> mCSVWriter2;

	private VisionSubsystem m_vision;

	private double desiredHeading;

	private final ProfiledPIDController headingController = new ProfiledPIDController(DriveConstants.kTurnP,
			DriveConstants.kTurnI, DriveConstants.kTurnD,
			new TrapezoidProfile.Constraints(VisionConstants.kMaxTurnVelocity, VisionConstants.kMaxTurnAcceleration));

	private final ProfiledPIDController visionDistanceController = new ProfiledPIDController(VisionConstants.kDriveP,
			VisionConstants.kDriveI, VisionConstants.kDriveD,
			new TrapezoidProfile.Constraints(VisionConstants.kDriveMaxSpeed, VisionConstants.kDriveMaxAcceleration));

	private boolean headingOverride = true;
	private boolean visionHeadingOverride = false;

	private final AnalogInput leftFrontAbsEncoder;
	private final AnalogInput rightFrontAbsEncoder;
	private final AnalogInput leftRearAbsEncoder;
	private final AnalogInput rightRearAbsEncoder;

	private double driveSpeedScaler = 1.0;

	private double flightTime = 1.3;

	private Double lockedHeading = null;
	private double m_heading;
	private boolean m_drivePolar = false;
	private double lastVisionTimestamp = 0;

	// Robot swerve modules
	private final SwerveModule m_leftFront;
	private final SwerveModule m_rightFront;
	private final SwerveModule m_leftRear;
	private final SwerveModule m_rightRear;
	private int _sdCount = 0;
	// private final SlewRateLimiter xfilter; // unused
	// private final SlewRateLimiter yfilter; // unused

	// The gyro sensor
	// private final Gyro a_gyro = new ADXRS450_Gyro();
	private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

	// Odometry class for tracking robot pose
	private SwerveDriveOdometry m_odometry = null; //new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getAngle(), null);
	private double _xVelocity;
	private double _yVelocity;
	private final Pose2d goalPose = new Pose2d(new Translation2d(8.188, 4.155), new Rotation2d(0));


	public void updateOdometry() {
		if (isDisabled())
			return;

		if (m_odometry != null) {
			SwerveModulePosition[] swerveModulePositions = null; //new SwerveModulePosition[]{m_leftFront.getState(), m_rightFront.getState(), m_leftRear.getState(), m_rightRear.getState()};
			// m_odometry.update(new Rotation2d(Math.toRadians(getHeading())), swerveModulePositions  // leftFront,
			// 																						// rightFront,
			// 																						// leftRear,
			// 																						// rightRear
			// 		);
		}

		var chassisState = DriveConstants.kDriveKinematics.toChassisSpeeds(m_leftFront.getState(), 
									m_rightFront.getState(), m_leftRear.getState(), m_rightRear.getState());

									

		_xVelocity = chassisState.vxMetersPerSecond;
		_yVelocity = chassisState.vyMetersPerSecond;	

	}

	/**
	 * Creates a new DriveSubsystem.
	 * 
	 * @param m_drive
	 */
	public DriveSubsystem(VisionSubsystem vision) {
		if (isDisabled()) {
			leftFrontAbsEncoder = null;
			rightFrontAbsEncoder = null;
			leftRearAbsEncoder = null;
			rightRearAbsEncoder = null;
			mCSVWriter1 = null;
			mCSVWriter2 = null;
			m_leftFront = null;
			m_rightFront = null;
			m_leftRear = null;
			m_rightRear = null;
			// xfilter = null;
			// yfilter = null;
			return;
		}

		m_leftFront = new SwerveModule(DriveConstants.kLeftFrontDriveMotorPort,
				DriveConstants.kLeftFrontTurningMotorPort);
		m_rightFront = new SwerveModule(DriveConstants.kRightFrontDriveMotorPort,
				DriveConstants.kRightFrontTurningMotorPort);
		m_leftRear = new SwerveModule(DriveConstants.kLeftRearDriveMotorPort,
				DriveConstants.kLeftRearTurningMotorPort);
		m_rightRear = new SwerveModule(DriveConstants.kRightRearDriveMotorPort,
				DriveConstants.kRightRearTurningMotorPort);

		 //xfilter = new SlewRateLimiter(100); // unused

		 //yfilter = new SlewRateLimiter(100); // unused

		leftFrontAbsEncoder = new AnalogInput(0);
		rightFrontAbsEncoder = new AnalogInput(1);
		leftRearAbsEncoder = new AnalogInput(2);
		rightRearAbsEncoder = new AnalogInput(3);

		if (RobotBase.isReal()) {
			if (leftFrontAbsEncoder == null || rightFrontAbsEncoder == null || leftRearAbsEncoder == null
					|| rightRearAbsEncoder == null) {
				System.err.println("\n\nAt least one absolute encoder (AnalogInput(0)--AnalogInput(3) is NULL!!!\n\n");
			}
		}
		headingController.setTolerance(VisionConstants.kTurnToleranceDeg, VisionConstants.kTurnRateToleranceDegPerS);
		visionDistanceController.setTolerance(VisionConstants.kDriveTolerance,
				VisionConstants.kDriveAccelerationTolerance);
		visionDistanceController.setGoal(0);
		headingController.enableContinuousInput(-180, 180);
		m_vision = vision;
		mCSVWriter1 = new ReflectingCSVWriter<>(AutoSwerveDebug.class);
		mCSVWriter2 = new ReflectingCSVWriter<>(SwerveModuleDebug.class);
		m_timer.reset();
		m_timer.start();

		// PID tuning
		/*
		 * SmartDashboard.putNumber("VisionP", VisionConstants.kTurnP);
		 * SmartDashboard.putNumber("VisionI", VisionConstants.kTurnI);
		 * SmartDashboard.putNumber("VisionD", VisionConstants.kTurnD);
		 */

	}

	public void setDriveSpeedScaler(double axis) {
		if (isDisabled())
			return;

		this.driveSpeedScaler = 0.5 * (axis + 1);
	}

	public ReflectingCSVWriter<AutoSwerveDebug> getCSVWriter() {
		if (isDisabled())
			return null;

		return mCSVWriter1;
	}

	/**
	 * Returns the angle of the robot as a Rotation2d.
	 *
	 * @return The angle of the robot.
	 */
	public Rotation2d getAngle() {
		if (isDisabled())
			return Rotation2d.fromDegrees(0);

		// Negating the angle because WPILib gyros are CW positive.
		return Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0));
	}

	@Override
	public void periodic() {
		if (isDisabled())
			return;

		// resumeCSVWriter();

		// Update the odometry in the periodic block
		// updateOdometry();


		// mCSVWriter2.add(new SwerveModuleDebug(m_timer.get(),
		// m_leftFront.getDebugValues(),
		// m_rightFront.getDebugValues(), m_leftRear.getDebugValues(),
		// m_rightRear.getDebugValues()));
	}
public void doSD() {
	if (_sdCount++ > 50) {
		SmartDashboard.putNumber("pose x", m_odometry.getPoseMeters().getTranslation().getX());
		SmartDashboard.putNumber("pose y", m_odometry.getPoseMeters().getTranslation().getY());
		SmartDashboard.putNumber("rot deg", m_odometry.getPoseMeters().getRotation().getDegrees());
		// SmartDashboard.putNumber("heading radians", Math.toRadians(getHeading()));
		// SmartDashboard.putNumber("raw gyro", m_gyro.getAngle());
		// SmartDashboard.putBoolean("gyro is calibrating", m_gyro.isCalibrating());
		SmartDashboard.putNumber("Gyro", m_heading);
		SmartDashboard.putNumber("TargetAngleFromVision", m_vision.getLastTarget().getY());
		SmartDashboard.putNumber("TargetDistanceFromVision", m_vision.getLastTarget().getTargetDistance());
		SmartDashboard.putNumber("ApproximateHubAngle", getApproximateHubAngle());
		SmartDashboard.putNumber("ApproximateHubDistance", getApproximateHubDistance());
		SmartDashboard.putBoolean("ApproximationStale", approximationStale());
		SmartDashboard.putNumber("leftFrontAbsEncoder", leftFrontAbsEncoder.getVoltage()); // 0.0 to 3.26, 180=1.63V
		SmartDashboard.putNumber("rightFrontAbsEncoder", rightFrontAbsEncoder.getVoltage()); // 0.0 to 3.26,
																								// 180=1.63V
		SmartDashboard.putNumber("leftRearAbsEncoder", leftRearAbsEncoder.getVoltage()); // 0.0 to 3.26, 180=1.63V
		SmartDashboard.putNumber("rightRearAbsEncoder", rightRearAbsEncoder.getVoltage()); // 0.0 to 3.26, 180=1.63V

		SmartDashboard.putBoolean("DrivePolar", m_drivePolar);

		// SmartDashboard.putNumber("leftFrontRelEncoder", m_leftFront.m_turningMotor.getSelectedSensorPosition());
		// SmartDashboard.putNumber("rightFrontRelEncoder", m_rightFront.m_turningMotor.getSelectedSensorPosition());
		// SmartDashboard.putNumber("leftRearRelEncoder", m_leftRear.m_turningMotor.getSelectedSensorPosition());
		// SmartDashboard.putNumber("rightRearRelEncoder", m_rightRear.m_turningMotor.getSelectedSensorPosition());

		// SmartDashboard.putNumber("LF drive Position",
		// m_leftFront.m_driveMotor.getSelectedSensorPosition(0));
		// SmartDashboard.putNumber("LF drive Velocity",
		// m_leftFront.m_driveMotor.getSelectedSensorVelocity(0));
		// SmartDashboard.putNumber("LF turn Position",
		// m_leftFront.m_turningMotor.getSelectedSensorPosition(0));
		// SmartDashboard.putNumber("LF turn Velocity",
		// m_leftFront.m_turningMotor.getSelectedSensorVelocity(0));
		// SmartDashboard.putNumber("LF speed m/s",
		// m_leftFront.getState().speedMetersPerSecond);
		// SmartDashboard.putNumber("LF azimuth",
		// m_leftFront.getState().angle.getDegrees());
		// SmartDashboard.putNumber("RF turn Position",
		// m_rightFront.m_turningMotor.getSelectedSensorPosition(0));
		// SmartDashboard.putNumber("LR turn Position",
		// m_leftRear.m_turningMotor.getSelectedSensorPosition(0));
		// SmartDashboard.putNumber("RR turn Position",
		// m_rightRear.m_turningMotor.getSelectedSensorPosition(0));
		_sdCount = 0; 
	   }
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		if (isDisabled())
			return new Pose2d();

		return m_odometry.getPoseMeters();
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		if (isDisabled())
			return;

		// m_odometry.resetPosition(pose, getAngle());
	}

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 */
	/*
	 * @SuppressWarnings("ParameterName")
	 * public void drive(double xSpeed, double ySpeed, double rot, boolean
	 * fieldRelative) {
	 * if(isDisabled()) return;
	 * 
	 * drive(xSpeed, ySpeed, 0, 0, fieldRelative);
	 * }
	 */

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative
	 * @param polardrive    Whether we are driving in polar coordinates around to
	 *                      goal
	 */
	@SuppressWarnings("ParameterName")
	public void drive(double xSpeed, double ySpeed, double rightX, double rightY, boolean fieldRelative,
			boolean fieldPolar) {
		if (isDisabled()) {
			return;
		}
		// System.out.println("drivepolar" + m_drivePolar);
		m_drivePolar = fieldPolar;
		// SmartDashboard.putBoolean("DrivePolar", m_drivePolar);

		if ((fieldPolar)) {
			updateVisionOdometry();

			if (!approximationStale()) {
				setVisionHeadingGoal(getApproximateHubAngle());
				setVisionHeadingOverride(true);
			} else {
				setVisionHeadingOverride(false);
			}
		} else {
			m_vision.disableLED(false);
			setVisionHeadingOverride(false);
		}

		double xSpeedAdjusted = xSpeed;
		double ySpeedAdjusted = ySpeed;
		// double rotAdjusted = rightX;
		double rotationalOutput = rightX;

		// DEADBAND
		if (Math.abs(xSpeedAdjusted) < 0.1) {
			xSpeedAdjusted = 0;
		}
		if (Math.abs(ySpeedAdjusted) < 0.1) {
			ySpeedAdjusted = 0;
		}
		/*
		 * if(Math.abs(rotAdjusted) < 0.3){ rotAdjusted = 0; }
		 */
		xSpeedAdjusted *= this.driveSpeedScaler;
		ySpeedAdjusted *= this.driveSpeedScaler;

	   // xSpeedAdjusted = xfilter.calculate(xSpeedAdjusted);
		//ySpeedAdjusted = yfilter.calculate(ySpeedAdjusted);



		// If the right stick is neutral - this code should lock on the last known
		// heading
		if (Math.abs(rotationalOutput) < 0.11) {
			headingOverride = true;
			if (lockedHeading == null) {
				headingController.reset(getHeading());
				desiredHeading = getHeading();
				lockedHeading = desiredHeading;
			} else {
				desiredHeading = lockedHeading;
			}
		} else {
			headingOverride = false;
			lockedHeading = null;
			rotationalOutput *= Math.PI;
		}

		if (visionHeadingOverride || headingOverride) {

			if (visionHeadingOverride) {
				rotationalOutput = headingController.calculate(getHeading());
				desiredHeading = getHeading();
				lockedHeading = getHeading();
				// SmartDashboard.putNumber("headingController Output", rotationalOutput);
			} else {
				// headingController.reset(getHeading());
				// desiredHeading += rotationalOutput*2.5;
				rotationalOutput = headingController.calculate(getHeading(), desiredHeading);
			}
		}

		/*
		 * if(Math.abs(rotationalOutput) < 0.1){ rotationalOutput = 0; }
		 */
		/*
		 * if (visionDistanceOverride && m_vision != null) {
		 * // This allows the driver to still have forward/backward control of the robot
		 * // while getting to optimal shooting in case something is in the way
		 * xSpeedAdjusted = Utils
		 * .Clamp(xSpeedAdjusted +
		 * visionDistanceController.calculate(m_vision.getLastTarget().getZ()), 0, 1);
		 * SmartDashboard.putNumber("distanceController Output", ySpeedAdjusted);
		 * } else if (m_vision != null) {
		 * visionDistanceController.reset(m_vision.getLastTarget().getZ());
		 * }
		 */
		var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedAdjusted, ySpeedAdjusted, rotationalOutput, getAngle())
				: new ChassisSpeeds(xSpeedAdjusted, ySpeedAdjusted, rotationalOutput));
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
		// SmartDashboard.putNumber("SwerveModuleAzimuthState",
		// swerveModuleStates[0].angle.getDegrees());
		m_leftFront.setDesiredState(swerveModuleStates[0]); // leftFront, rightFront, leftRear, rightRear
		m_rightFront.setDesiredState(swerveModuleStates[1]);
		m_leftRear.setDesiredState(swerveModuleStates[2]);
		m_rightRear.setDesiredState(swerveModuleStates[3]);

	}

	/**
	 * Sets the swerve ModuleStates.
	 *
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		if (isDisabled())
			return;

		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
		m_leftFront.setDesiredState(desiredStates[0]); // leftFront, rightFront, leftRear, rightRear
		m_rightFront.setDesiredState(desiredStates[1]);
		m_leftRear.setDesiredState(desiredStates[2]);
		m_rightRear.setDesiredState(desiredStates[3]);
	}

	/**
	 * Resets the drive encoders to currently read a position of 0.
	 */
	public void resetEncoders() {
		if (isDisabled())
			return;

		m_leftFront.resetEncoders(leftFrontAbsEncoder.getVoltage() / 3.3); // leftFront, rightFront, leftRear,
																			// rightRear
		m_rightFront.resetEncoders(rightFrontAbsEncoder.getVoltage() / 3.3);// nope! took it back out!// had taken out
																				// but it started working again
																				// 7mar2020. // took this one out -- bad
																				// hardware encoder!!!
		// m_rightFront.resetEncoders(0);// had taken out but it started working again
		// 7mar2020. // took this one out -- bad hardware encoder!!!
		m_leftRear.resetEncoders(leftRearAbsEncoder.getVoltage() / 3.3);
		m_rightRear.resetEncoders(rightRearAbsEncoder.getVoltage() / 3.3);
		resetOdometry(new Pose2d());

	}

	/**
	 * Zeroes the heading of the robot.
	 */
	public void zeroHeading() {
		if (isDisabled())
			return;

		m_gyro.reset(); // RDB2020 - I replace this call with the below 5 lines...
	    m_gyro.setAngleAdjustment(0.0);
	}

	public void setAngleAdjustment(double angle) {
		m_gyro.setAngleAdjustment(angle);
	}
    

	/**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from -180 to 180
	 */
	public double getHeading() {
		if (isDisabled())
			return 0;

		if (m_gyro != null) {
			m_heading = Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);

		}
		return m_heading;
	}

	public double getXaccel() {
		if (isDisabled())
			return 0;

		return m_gyro.getWorldLinearAccelX() / 9.8066;
	}

	public double getYaccel() {
		if (isDisabled())
			return 0;

		return m_gyro.getWorldLinearAccelY() / 9.8066;
	}

	/**
	 * Returns the turn rate of the robot.
	 *
	 * @return The turn rate of the robot, in degrees per second
	 */
	public double getTurnRate() {
		if (isDisabled())
			return 0;

		return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
	}

	public void suspendCSVWriter() {
		if (isDisabled())
			return;

		if (!mCSVWriter1.isSuspended()) {
			mCSVWriter1.suspend();
		}
		if (!mCSVWriter2.isSuspended()) {
			mCSVWriter2.suspend();
		}
	}

	public void resumeCSVWriter() {
		if (isDisabled())
			return;

		if (mCSVWriter1.isSuspended()) {
			mCSVWriter1.resume();
		}
		if (mCSVWriter2.isSuspended()) {
			mCSVWriter2.resume();
		}
	}

	/**
	 * Determines if the right thumbstick on the driver controller can control the
	 * robot's orientation. Set to false if you want another subsystem to control
	 * the orientation in teleop (e.x. vision)
	 */
	public void setVisionHeadingOverride(boolean visionOverride) {
		if (isDisabled())
			return;

		visionHeadingOverride = visionOverride;
		headingController.setP(visionOverride ? VisionConstants.kTurnP : DriveConstants.kTurnP);
		headingController.setI(visionOverride ? VisionConstants.kTurnI : DriveConstants.kTurnI);
		headingController.setD(visionOverride ? VisionConstants.kTurnD : DriveConstants.kTurnD);
	}

	public void setVisionDistanceOverride(boolean visionOverride) {
		if (isDisabled())
			return;

		// visionDistanceOverride = visionOverride;
	}

	public void setVisionHeadingGoal(double newGoal) {
		if (isDisabled())
			return;

		headingController.setGoal(newGoal);
	}

	public void resetGyro() {
		if (isDisabled())
			return;

		if (m_gyro != null) {
			desiredHeading = 0;
			lockedHeading = null;
			m_gyro.reset();
			m_gyro.setAngleAdjustment(0.0);
		}
	}

	public double getXVelocity() {
		return 0;
	}

	public double getYVelocity() {
		return 0;
	}

	public void updateVisionOdometry() {
		m_vision.enableLED();
		if (m_vision.hasTarget()) {
			//System.out.println("inUpdateVisionOdometry");
			// determine position on the field and set odometry
			double y = (4.15 - (m_vision.getLastTarget().getTargetDistance() + .67)
					* Math.sin(Math.toRadians(getHeading()) - Math.toRadians(m_vision.getLastTarget().getY())));
			double x = (8.188 - (m_vision.getLastTarget().getTargetDistance() + .67)
					* Math.cos(Math.toRadians(getHeading()) - Math.toRadians(m_vision.getLastTarget().getY())));
			resetOdometry(new Pose2d(new Translation2d(x, y), new Rotation2d(Math.toRadians(getHeading()))));
			lastVisionTimestamp = m_vision.getLastTarget().getTimeCaptured();
		}

	}

	// public void setAngleOffsetDegrees(double degrees) {
	// m_gyro.setAngleAdjustment(degrees);
	// }

	public double getApproximateHubDistance() {
		var newGoalPose = goalPose.exp(
			new Twist2d(
		   _xVelocity * flightTime * -1,
		   _yVelocity * flightTime * -1,
		   Math.toRadians(getHeading())));
		return Math.sqrt(Math.pow(newGoalPose.getY() - m_odometry.getPoseMeters().getY(), 2)
				+ Math.pow(newGoalPose.getX() - m_odometry.getPoseMeters().getX(), 2));
	}

	public double getApproximateHubAngle() {
		
		var newGoalPose = goalPose.exp(
			 new Twist2d(
			_xVelocity * flightTime * -1.0,
			_yVelocity * flightTime * -1.0,
			Math.toRadians(getHeading())));

		
		double x = m_odometry.getPoseMeters().getX();
		double angle = Math.toDegrees(Math.atan(((newGoalPose.getY()) - m_odometry.getPoseMeters().getY()) / ((newGoalPose.getX()) - x)));
		if (x  > newGoalPose.getX()) {
			angle = angle - 180;
		}
		return angle;

	}

	public boolean approximationStale() {
		return (Timer.getFPGATimestamp() - lastVisionTimestamp >= 15 );
	}

	public void setCurrentLimits(boolean enable) {
		m_leftFront.setCurrentLimits(enable);
		m_rightFront.setCurrentLimits(enable);
		m_leftRear.setCurrentLimits(enable);
		m_rightRear.setCurrentLimits(enable);
	
	}

	public void allStop() {
		m_leftFront.allStop();
		m_rightFront.allStop();
		m_leftRear.allStop();
		m_rightRear.allStop();
	}

}
