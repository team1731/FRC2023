/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.Constants.kTICKS;

public class SwerveModule {
	public static final double kMaxAngularSpeed = Math.PI;
	public TalonFX m_driveMotor;
	public TalonFX m_turningMotor;

	private int id;
	private Boolean isInverted = Boolean.FALSE;
	private DebugValues debugValues;

	/**
	 * Constructs a SwerveModule.
	 *
	 * @param driveMotorChannel   ID for the drive motor.
	 * @param turningMotorChannel ID for the turning motor.
	 */
	public SwerveModule(int driveMotorChannel, int turningMotorChannel) {
		id = driveMotorChannel;
		debugValues = new DebugValues(id);

		if (RobotBase.isReal()) {

			m_driveMotor = new WPI_TalonFX(driveMotorChannel);
		//	m_driveMotor.configFactoryDefault();

			m_driveMotor.setInverted(true);


            
			// m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
			// m_driveMotor.config_kP(0, 0.1, 30);
			// m_driveMotor.config_kI(0, 0, 30);
			// m_driveMotor.config_kD(0, 0, 30);
			// m_driveMotor.config_kF(0, 1023.0 / 20660.0, 30);

			// /* Config neutral deadband to be the smallest possible */
			// m_driveMotor.configNeutralDeadband(0.001);

			// /* Config the peak and nominal outputs */
			// m_driveMotor.configNominalOutputForward(0, 30);
			// m_driveMotor.configNominalOutputReverse(0, 30);
			// m_driveMotor.configPeakOutputForward(1, 30);
			// m_driveMotor.configPeakOutputReverse(-1, 30);

			m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5,30);


			m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 170,30);
			m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 241, 30);
			m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 241, 30);
			m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 235, 30);
			m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc,190, 30);
			m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer,239, 30);
			m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer,257, 30);
			m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,168,30);
			m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 130, 30);
			m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 210, 30);
			m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus,178, 30);
			m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 50, 30);
			m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 247, 30);

			m_turningMotor = new WPI_TalonFX(turningMotorChannel);
			setCurrentLimits(false);

			/* Factory default hardware to prevent unexpected behavior */
			//m_turningMotor.configFactoryDefault();

			/* Configure Sensor Source for Pirmary PID */
			m_turningMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

			/*
			 * set deadband to super small 0.001 (0.1 %). The default deadband is 0.04 (4 %)
			 */
			m_turningMotor.configNeutralDeadband(0.001, 30);

			/**
			 * Configure Talon FX Output and Sesnor direction accordingly Invert Motor to
			 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
			 * sensor to have positive increment when driving Talon Forward (Green LED)
			 */
			m_turningMotor.setSensorPhase(false);
			m_turningMotor.setInverted(true);
			/*
			 * Talon FX does not need sensor phase set for its integrated sensor This is
			 * because it will always be correct if the selected feedback device is
			 * integrated sensor (default value) and the user calls getSelectedSensor* to
			 * get the sensor's position/velocity.
			 * 
			 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
			 * sensor-phase
			 */
			// m_turningMotor.setSensorPhase(true);

			/* Set relevant frame periods to be at least as fast as periodic rate */
			m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
			m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, 30);
			m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 30);
			
			m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 159,30);
			m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 231, 30);
			m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 250, 30);
			m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 241, 30);
			m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc,150, 30);
			m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer,234, 30);
			m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer,247, 30);
			m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,10,30);
			m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 242, 30);
			m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 253, 30);
			m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus,254, 30);
			
		

			// /* Set the peak and nominal outputs */
			// m_turningMotor.configNominalOutputForward(0, 30);
			// m_turningMotor.configNominalOutputReverse(0, 30);
			// m_turningMotor.configPeakOutputForward(1, 30);
			// m_turningMotor.configPeakOutputReverse(-1, 30);

			// /* Set Motion Magic gains in slot0 - see documentation */
			// m_turningMotor.selectProfileSlot(0, 0);
			// m_turningMotor.config_kF(0, 0.2, 30);
			// m_turningMotor.config_kP(0, 0.2, 30);
			// m_turningMotor.config_kI(0, 0, 30);
			// m_turningMotor.config_kD(0, 0, 30);

			// /* Set acceleration and vcruise velocity - see documentation */
			// m_turningMotor.configMotionCruiseVelocity(18000, 0);
			// m_turningMotor.configMotionAcceleration(18000, 0);
			// m_turningMotor.configMotionSCurveStrength(2);

			/* Zero the sensor once on robot boot up */
			m_turningMotor.setSelectedSensorPosition(0, 0, 30);


		} else {
			m_driveMotor = null;
			m_turningMotor = null;
		}

	}

	public SwerveModule() {
		System.err.println("DUMMY SWERVE MODULE HAS BEEN INSTANTIATED");
	}

	/**
	 * Returns the current state of the module.
	 *
	 * @return The current state of the module.
	 */
	public SwerveModuleState getState() {
		// return new SwerveModuleState(m_driveEncoder.getRate(), new
		// Rotation2d(m_turningEncoder.get()));
		double velocity = 0;
		double azimuth = 0;
		if (RobotBase.isReal()) { // RPM/60 is RPS *PI*D is inches/s * 39.37 is meter/s but it's 5.5 ticks/rev
			velocity = (m_driveMotor.getSelectedSensorVelocity(0) / 204.8 * Math.PI * 3.0) / (39.37 * 4.6666666666);
			azimuth = -m_turningMotor.getSelectedSensorPosition(0);
		}
		double azimuthPercent = Math.IEEEremainder(azimuth, kTICKS) / kTICKS;

		if (RobotBase.isReal()) {
			// SmartDashboard.putNumber("Module"+id+" Drive Encoder Tick",
			// m_driveEncoder.getPosition());
		}

		return new SwerveModuleState(velocity, new Rotation2d(azimuthPercent * 2.0 * Math.PI));

	}

	/**
	 * Sets the desired state for the module.
	 *
	 * @param state Desired state with speed and angle.
	 */
	public void setDesiredState(SwerveModuleState state) {

		double azimuth = -state.angle.getDegrees() * kTICKS / 360.0;
		double speedMetersPerSecond = state.speedMetersPerSecond;
		// SmartDashboard.putNumber("SpeedMPS-"+id, speedMetersPerSecond);
		// meters per sec * 39.37 is inches/s * 60 is inches per min / PI*D is RPM * 5.5
		// is ticks
		double drive = (speedMetersPerSecond * 9557.333333 * 39.37) / (3.0 * Math.PI); // ticks per second
		// wheel.set(-angleDegrees/360, speedMetersPerSecond * 16.0 * 39.37 * 60.0 / 3.0
		// / Math.PI);
		double azimuthPosition = 0;
		if (RobotBase.isReal()) {
			// azimuthPosition = m_turningEncoder.getPosition();
			azimuthPosition = m_turningMotor.getSelectedSensorPosition(0);
		}
		double azimuthError = Math.IEEEremainder(azimuth - azimuthPosition, kTICKS);

		// ********************************************************
		// minimize azimuth rotation, reversing drive if necessary
		// ********************************************************
		// synchronized(isInverted){
		isInverted = Math.abs(azimuthError) > 0.25 * kTICKS;
		if (isInverted) {
			azimuthError -= Math.copySign(0.5 * kTICKS, azimuthError);
			drive = -drive;
		}
		// }

		if (RobotBase.isReal()) {
			double turningMotorOutput = azimuthPosition + azimuthError;
			// m_turningPIDController.setReference(turningMotorOutput,
			// ControlType.kSmartMotion);
			m_turningMotor.set(TalonFXControlMode.MotionMagic, turningMotorOutput);
			// m_drivePIDController.setReference(drive, ControlType.kSmartVelocity);
			double targetVelocity_UnitsPer100ms = drive / 10; // ticks per 100 ms
			/* 500 RPM in either direction */
			m_driveMotor.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);

		//	if (System.currentTimeMillis() % 100 == 0) {
		//		SmartDashboard.putNumber("turningMotorOutput-" + id, turningMotorOutput);
		//		SmartDashboard.putNumber("driveVelocityOutput-" + id, drive);
		//	}

		//	debugValues.update(drive, turningMotorOutput, m_turningMotor.getMotorOutputPercent(),
		//			m_turningMotor.getSelectedSensorVelocity(0), m_driveMotor.getMotorOutputPercent(),
		//			m_driveMotor.getSelectedSensorPosition());
		}

		// SmartDashboard.putNumber("RelativeEncoder"+id,
		// m_turningEncoder.getPosition());
		// SmartDashboard.putNumber("absOffset"+id, offsetFromAbsoluteEncoder);
	}

	public void setCurrentLimits(boolean enable) {

		SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(enable, 40, 60, 5);
		StatorCurrentLimitConfiguration statorLimit = new StatorCurrentLimitConfiguration(enable, 60, 50, 5);

		m_driveMotor.configSupplyCurrentLimit(limit,0);
		m_driveMotor.configStatorCurrentLimit(statorLimit,0);
	}

	/**
	 * Zeros all the SwerveModule encoders.
	 */
	public void resetEncoders(double absoluteEncoderVoltage) {
		// synchronized(isInverted){
		if (RobotBase.isReal()) {
			m_driveMotor.setSelectedSensorPosition(0, 0, 0);
			// m_driveEncoder.setPosition(0);
			// m_turningEncoder.setPosition(absoluteEncoderVoltage * 16/3.26);
			// absoluteEncoderVoltage = 0;
			m_turningMotor.setSelectedSensorPosition(absoluteEncoderVoltage * kTICKS, 0, 0);
		}
	}
	// }

	//public DebugValues getDebugValues() {
	//	return debugValues;
	//}

	public class DebugValues {
		public int id;

		public double drive;
		public double turningMotorOutput;
		public double turnAppliedOutput;
		public double turnVelocity;
		public double driveAppliedOutput;
		public double driveVelocity;

		public DebugValues(int id) {
			this.id = id;
		}

		public void update(double drive, double turningMotorOutput, double turnAppliedOutput, double turnVelocity,
				double driveAppliedOutput, double driveVelocity) {
			this.drive = drive;
			this.turningMotorOutput = turningMotorOutput;
			this.turnAppliedOutput = turnAppliedOutput;
			this.turnVelocity = turnVelocity;
			this.driveAppliedOutput = driveAppliedOutput;
			this.driveVelocity = driveVelocity;
		}
	}

    public void allStop() {
		m_driveMotor.set(TalonFXControlMode.PercentOutput, 0);
		m_turningMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

}
