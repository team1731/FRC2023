package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
    public int moduleNumber;
    private double angleOffset;
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;
    private double lastAngle;
    private int falcontics = 0;
    private DebugValues debugValues;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        debugValues = new DebugValues(moduleNumber);
        angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID,"canivore1");
        configAngleEncoder();
    

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID,"canivore1");
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID,"canivore1");
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean lockAngle){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) && !lockAngle ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.

        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.Swerve.angleGearRatio)); 
        lastAngle = angle;
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, Constants.Swerve.angleGearRatio);
        System.err.println("Cancoder degrees" + getCanCoder().getDegrees());
        System.err.println("angleoffset" + angleOffset);
        System.err.println("absolutePosition" + absolutePosition);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState(){
        double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition(){
        double position = ((mDriveMotor.getSelectedSensorPosition()/2048.0) * Constants.Swerve.wheelCircumference) / Constants.Swerve.driveGearRatio;
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
        return new SwerveModulePosition(position, angle);
    }
    
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
}