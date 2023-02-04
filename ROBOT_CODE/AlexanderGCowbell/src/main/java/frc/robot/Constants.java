package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.state.StateChangeRequest;
import frc.robot.state.arm.ArmStateMachine.ArmInput;

public final class Constants {
    public static final double stickDeadband = 0.1;
	public static final int kTICKS = 33024; // 16.125 * 2048;

    public static final class FieldConstants {
        // Note: Field dimensions and April Tag positions pulled from the 2023 Field and Layout Marking document
        public static final double kFieldLength = Units.inchesToMeters(651.22);
        public static final double kFieldWidth = Units.inchesToMeters(315.1);

        public static class AprilTagPoseValues {
            public int id;
            public double x;
            public double y;
            public double z;
            public double yaw;

            public AprilTagPoseValues(int tagId, double xInches, double yInches, double zInches, double yawDegrees) {
                id = tagId;
                x = Units.inchesToMeters(xInches);
                y = Units.inchesToMeters(yInches);
                z = Units.inchesToMeters(zInches);
                yaw = Units.degreesToRadians(yawDegrees);
            }

            public AprilTag getAprilTag() {
                return new AprilTag(id, new Pose3d(x, y, z, new Rotation3d(0.0, 0.0, yaw)));
            }
        }

        public static final AprilTagPoseValues kAprilTagPose1 = new AprilTagPoseValues(1, 610.77, 42.19, 18.22, 180);
        public static final AprilTagPoseValues kAprilTagPose2 = new AprilTagPoseValues(2, 610.77, 108.19, 18.22, 180);
        public static final AprilTagPoseValues kAprilTagPose3 = new AprilTagPoseValues(3, 610.77, 147.19, 18.22, 180);
        public static final AprilTagPoseValues kAprilTagPose4 = new AprilTagPoseValues(4, 636.96, 265.74, 27.38, 180);
        public static final AprilTagPoseValues kAprilTagPose5 = new AprilTagPoseValues(5, 14.25, 265.74, 27.38, 0);
        public static final AprilTagPoseValues kAprilTagPose6 = new AprilTagPoseValues(6, 40.45, 147.19, 18.22, 0);
        public static final AprilTagPoseValues kAprilTagPose7 = new AprilTagPoseValues(7, 40.45, 108.19, 18.22, 0);
        public static final AprilTagPoseValues kAprilTagPose8 = new AprilTagPoseValues(8, 40.45, 42.19, 18.22, 0);
    }

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
        public static final double maxSpeed = 0.5; // disabled for testing = 4.5; //meters per second
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

		public static final double kMaxSpeedMetersPerSecond = 0.5; // disabled for testing = 3.5; // tune

		public static final double kTurnP = 0.1; // was 0.05
		public static final double kTurnI = 0;
		public static final double kTurnD = 0;

	}

    public static final class AutoConstants {
        public static final String kDefault                      = "_Default_Auto";
        public static final String k_0_Example                   = "_0_Example_Auto";
        public static final String k_1_11Top_A_13Top_Drive_A     = "_1_1Top_A_13Top_Drive_A";
        public static final String k_2_13Top_B_Engage            = "_2_13Top_B_Engage";
        public static final String k_3_31Top_C_Engage            = "_3_31Top_C_Engage";
        public static final String k_4_33Top_D_31Top_Drive_D     = "_4_33Top_D_31Top_Drive_D";
        public static final String k_5_11Top_A_11Middle_Drive_A  = "_5_11Top_A_11Middle_Drive_A";
        public static final String k_6_33Top_D_33Middle_Drive_D  = "_6_33Top_D_33Middle_Drive_D";
        public static final String k_9_Move_Forward              = "_9_Move_Forward";
        
		public static final String kAutoCodeKey = "Auto Selector";
        public static final double kMaxSpeedMetersPerSecond = 0.5; // disabled for testing = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.5; // disabled for testing = 3;
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
      
    public static final class StateConstants {
        public static final String kSuccessCode = "00";
        public static final String kGenericFailedCode = "01";

        public enum StateMachineWaitCondition {
            UNTIL_LINED_UP_FOR_SCORING
        }
        
        /*
        * Constants specifically related to the ArmStateMachine
        */
        public static final String kArmStateMachineId = "ArmStateMachine";

        /*
         * Sequences for the ArmStateMachine
         * Note: leave test sequences in place, they are used by the ArmStateMachineTest (JUnit)
         */
        public static final StateChangeRequest[] kTestSequenceScore = new StateChangeRequest[]{
            new StateChangeRequest(ArmInput.EXTEND, new double[]{ 1, 2, 3, 4, 5 }),
            new StateChangeRequest(ArmInput.RELEASE, null, StateMachineWaitCondition.UNTIL_LINED_UP_FOR_SCORING),
            new StateChangeRequest(ArmInput.RETRACT, new double[]{ 6, 7, 8, 9, 10 })
        };

        public static final StateChangeRequest[] kTestSequencePickup = new StateChangeRequest[]{
            new StateChangeRequest(ArmInput.EXTEND, new double[]{ 1, 2, 3, 4, 5 }),
            new StateChangeRequest(ArmInput.INTAKE),
            new StateChangeRequest(ArmInput.RETRACT, new double[]{ 6, 7, 8, 9, 10 })
        };

        public static final StateChangeRequest[] kTestInvalid = new StateChangeRequest[]{
            new StateChangeRequest(ArmInput.EXTEND),
            new StateChangeRequest(ArmInput.EXTEND)
        };
    }

    public static final class VisionConstants {
		// Ensure measurements are in METERS
		public static final double kMaxDistanceBetweenPoseEstimations = 1.0;

        // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much you trust your various sensors. 
        // Smaller numbers will cause the filter to "trust" the estimate from that particular component more than the 
        // others. This in turn means the particular component will have a stronger influence on the final pose estimate.
        public static final Matrix<N3, N1> kVisionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(180));

        public static class CameraMountPoseValues {
            public String id;
            public double x;
            public double y;
            public double z;
            public double yaw;

            // Note: this constructor assumes the camera is mounted parallel to the floor
            public CameraMountPoseValues(String cameraId, double xInches, double yInches, double zInches, double yawDegrees) {
                id = cameraId;
                x = Units.inchesToMeters(xInches);
                y = Units.inchesToMeters(yInches);
                z = Units.inchesToMeters(zInches);
                yaw = Units.degreesToRadians(yawDegrees);
            }

            public Transform3d getPoseTransform() {
                return new Transform3d(new Translation3d(x, y, z), new Rotation3d(0.0,0.0,yaw));
            }
        }

        public static final String kCameraMount1Id = "Global_Shutter_Camera";
        public static final String kCameraMount2Id = "Microsoft_LifeCam_HD-3000";
        public static final CameraMountPoseValues kCameraMount1Pose = new CameraMountPoseValues(kCameraMount1Id, 15.5, 5.0, 39.37, 315);
        public static final CameraMountPoseValues kCameraMount2Pose = new CameraMountPoseValues(kCameraMount2Id, 15.5, 5.0, 39.37, 45);

		// #region TurnPID
		public static final double kTurnP = 0.12;
		public static final double kTurnI = 0;
		public static final double kTurnD = 0.01;
		public static final double kMaxTurnVelocity = 360;
		public static final double kMaxTurnAcceleration = 360;
		public static final double kTurnToleranceDeg = 5;
		public static final double kTurnRateToleranceDegPerS = 10; // degrees per second
		// #endregion
    }

    public static final class OpConstants{
        public static final int kPWM_LedSting = 6;         // Addressable Led String

        public enum LedOption {
            TEAM, RED, BLUE, GREEN, YELLOW, ORANGE, PURPLE, RAINBOW, FULL, CLIMB, SHOOT, INTAKE, INTAKEBALL, WHEEL, BALLONE, BALLTWO, BALLTHREE, BALLFOUR
          }
    }
}
