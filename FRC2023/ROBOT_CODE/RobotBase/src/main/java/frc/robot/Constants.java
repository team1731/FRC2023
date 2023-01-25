package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
	public static final int kTICKS = 33024; // 16.125 * 2048;

    public static final class Swerve {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.5);
        public static final double wheelBase = Units.inchesToMeters(26.75);
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.75 / 1.0); //6.86:1
        public static final double angleGearRatio = (150.0/ 7.0); //12.8:1  

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.6;
        public static final double angleKI = 0.0;
        public static final double angleKD = 12.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.10;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; //meters per second
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final double angleOffset = 219.375;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final double angleOffset = 27.42;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final double angleOffset = 266.66;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final double angleOffset = 135.70;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }

    public static final class DriveConstants {

		// Drive motor CAN IDs
		public static final int kLeftFrontDriveMotorPort = 21;
		public static final int kRightFrontDriveMotorPort = 22;
		public static final int kLeftRearDriveMotorPort = 23;
		public static final int kRightRearDriveMotorPort = 24;

		// Turn motor CAN IDs
		public static final int kLeftFrontTurningMotorPort = 11;
		public static final int kRightFrontTurningMotorPort = 12;
		public static final int kLeftRearTurningMotorPort = 13;
		public static final int kRightRearTurningMotorPort = 14;

		public static final double kTrackWidth = 0.7112;
		// Distance between centers of right and left wheels on robot
		public static final double kWheelBase = 0.7;
		// Distance between front and back wheels on robot
		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics( // leftFront,
																								// rightFront, leftRear,
																								// rightRear
				new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

		public static final boolean kGyroReversed = true; // 09FEB false;

		public static final double kMaxSpeedMetersPerSecond = 3.5; // tune

		public static final double kTurnP = 0.1; // was 0.05
		public static final double kTurnI = 0;
		public static final double kTurnD = 0;

	}

    public static final class AutoConstants {
        public static final String kDEFAULT_AUTO_CODE = "C4";
		public static final String kAUTO_CODE = "Auto Selector";
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }

      public static final class VisionConstants {
		// Ensure measurements are in METERS
		public static final double kMaxTrackerDistance = 18.0;
		public static final double kMaxGoalTrackAge = 1.0; // cp had 1.0
		public static final double kGoalHeight = 2.67;

		// Ensure measurements are in METERS
		public static final double kCameraXOffset = 0;
		public static final double kCameraYOffset = 0;
		public static final double kCameraZOffset = 0;
		public static final double kCameraPitchAngleDegrees = 41.67;
		public static final double kCameraYawAngleDegrees = 0;
		public static final double kCameraLensHeightMeters = 0.7142;

		// #region DrivePID
		public static final double kDriveP = 0.05;
		public static final double kDriveI = 0;
		public static final double kDriveD = 0;
		public static final double kDriveMaxSpeed = 0.1;
		public static final double kDriveMaxAcceleration = 0.1;
		public static final double kDriveTolerance = 0.5;
		public static final double kDriveAccelerationTolerance = 0.1;
		// #endregion

		// #region TurnPID
		public static final double kTurnP = 0.12;
		public static final double kTurnI = 0;
		public static final double kTurnD = 0.00;
		public static final double kMaxTurnVelocity = 360;
		public static final double kMaxTurnAcceleration = 360;
		public static final double kTurnToleranceDeg = 5;
		public static final double kTurnRateToleranceDegPerS = 10; // degrees per second
		// #endregion

		public static final double kAverageKeepTime = 0.2;
	}

}