package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Optional;
import java.util.Arrays;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.log.Logger;
import frc.robot.util.log.MessageLog;
import frc.robot.util.log.LogWriter;
import frc.robot.util.log.LogWriter.Log;
import frc.robot.util.log.loggers.PoseEstimations;


class CameraTransform {
    PhotonCamera camera;
    Transform3d transform;

    public CameraTransform(PhotonCamera cam, Transform3d trsfrm){
      camera = cam;
      transform = trsfrm;
    }
}

public class PoseEstimatorSubsystem extends SubsystemBase {

  private final Swerve m_swerve;
  private final SwerveDrivePoseEstimator poseEstimator;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private final Field2d field2d = new Field2d();
  private boolean useVisionCorrection = true;
  private boolean gamePieceDetected = false;
  private double cameraAngleOutFromFloor = Units.degreesToRadians(VisionConstants.CubeAutoAngle);
  private boolean aprilTagsIsEnabled = true;
  
  // Camera configuration
  private HashMap<String, CameraTransform> cameraMap = new HashMap<String, CameraTransform>();
  private final PhotonCamera photonCamera1 = new PhotonCamera(VisionConstants.kCameraMount1Id);
  private final PhotonCamera photonCamera2 = new PhotonCamera(VisionConstants.kCameraMount2Id);
  private final PhotonCamera photonCamera3 = new PhotonCamera(VisionConstants.kCameraMount3Id);

  // Physical location of the camera on the robot, relative to the center of the robot.
  private static final Transform3d CAMERA_TO_ROBOT_1 = VisionConstants.kCameraMount1Pose.getPoseTransform();
  private static final Transform3d CAMERA_TO_ROBOT_2 = VisionConstants.kCameraMount2Pose.getPoseTransform();
  private static final Transform3d CAMERA_TO_ROBOT_3 = VisionConstants.kCameraMount3Pose.getPoseTransform();

  // logging
  Logger poseLogger;
  double lastLogTime = 0;
  double logInterval = 1.0; // in seconds

  public PoseEstimatorSubsystem(Swerve m_swerve) {
    this.m_swerve = m_swerve;
    poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, m_swerve.getHeading(),
        m_swerve.getPositions(), new Pose2d());

    // Configure and map cameras using camera names and location on the robot
    if (photonCamera1 != null) {
      MessageLog.add("PoseEstimatorSubsystem: Adding vision measurement from " + VisionConstants.kCameraMount1Id);
      System.out.println(
          "PoseEstimatorSubsystem: Adding camera " + VisionConstants.kCameraMount1Id + "!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      this.cameraMap.put(VisionConstants.kCameraMount1Id, new CameraTransform(photonCamera1, CAMERA_TO_ROBOT_1));
    }
    if (photonCamera2 != null) {
      MessageLog.add("PoseEstimatorSubsystem: Adding vision measurement from " + VisionConstants.kCameraMount2Id);
      System.out.println(
          "PoseEstimatorSubsystem: Adding camera " + VisionConstants.kCameraMount2Id + "!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      this.cameraMap.put(VisionConstants.kCameraMount2Id, new CameraTransform(photonCamera2, CAMERA_TO_ROBOT_2));
    }

    if (photonCamera3 != null) {
      MessageLog.add("PoseEstimatorSubsystem: Adding vision measurement from " + VisionConstants.kCameraMount3Id);
      System.out.println(
          "PoseEstimatorSubsystem: Adding camera " + VisionConstants.kCameraMount3Id + "!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      this.cameraMap.put(VisionConstants.kCameraMount3Id, new CameraTransform(photonCamera3, CAMERA_TO_ROBOT_3));
    }
    // System.out.println("camera1:" +
    // cameraMap.get(photonCamera1.getName()).transform.getY() );
    // System.out.println("camera2:" +
    // cameraMap.get(photonCamera2.getName()).transform.getY() );
    // Define field dimensions and target positions for use with pose estimation
    AprilTag[] aprilTags = {
        FieldConstants.kAprilTagPose1.getAprilTag(),
        FieldConstants.kAprilTagPose2.getAprilTag(),
        FieldConstants.kAprilTagPose3.getAprilTag(),
        FieldConstants.kAprilTagPose4.getAprilTag(),
        FieldConstants.kAprilTagPose5.getAprilTag(),
        FieldConstants.kAprilTagPose6.getAprilTag(),
        FieldConstants.kAprilTagPose7.getAprilTag(),
        FieldConstants.kAprilTagPose8.getAprilTag(),
    };
    aprilTagFieldLayout = new AprilTagFieldLayout(Arrays.asList(aprilTags), FieldConstants.kFieldLength,
        FieldConstants.kFieldWidth);

    // write initial values to dashboard
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    tab.addString("Pose (X, Y)", this::getFomattedPose).withPosition(0, 4);
    tab.addNumber("Pose Degrees", () -> getCurrentPose().getRotation().getDegrees()).withPosition(1, 4);
    tab.add(field2d);

    // setup logger
    poseLogger = LogWriter.getLogger(Log.POSE_ESTIMATIONS, PoseEstimations.class);
  }

  @Override
  public void periodic() {
    // Loop through the cameras and update the pose estimator with visible targets
    if (aprilTagsIsEnabled) {
      for (String cameraName : this.cameraMap.keySet()) {

        PhotonCamera camera = this.cameraMap.get(cameraName).camera;

        if (!camera.isConnected() || camera.getName().equals(photonCamera3.getName())) {
          continue;
        }

        Transform3d robotToCamera = this.cameraMap.get(cameraName).transform;
        Pose2d currentPose = getCurrentPose();

        var visionResult = camera.getLatestResult();

        if (visionResult.hasTargets()) {
          PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
              PoseStrategy.AVERAGE_BEST_TARGETS, camera, robotToCamera);
          photonPoseEstimator.setReferencePose(currentPose);
          // System.out.println(camera.getName() + " USING X=" + robotToCamera.getX() + "
          // Y = " + robotToCamera.getY() + "Z = " + robotToCamera.getZ());
          Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update();
          if (estimatedPose.isPresent()) {
            EstimatedRobotPose cameraPose = estimatedPose.get();
            Pose2d cameraPose2d = cameraPose.estimatedPose.toPose2d();

            // determine difference between estimated pose and current pose
            Pose2d poseError = currentPose.relativeTo(cameraPose2d);
            double distanceBetweenPoseEstimations = Math
                .sqrt(Math.pow(poseError.getX(), 2) + Math.pow(poseError.getY(), 2));

            // If the distance between the vision pose estimation and the current swerve
            // pose estimation is too great we will
            // ignore it. This should help filter out bad vision data.
            // if(distanceBetweenPoseEstimations <
            // VisionConstants.kMaxDistanceBetweenPoseEstimations) {
            MessageLog.add("PoseEstimatorSubsystem: Adding vision measurement from " + cameraName);
            field2d.getObject("MyRobot" + cameraName).setPose(cameraPose2d);
            SmartDashboard.putString("Vision pose", String.format("(%.2f, %.2f) %.2f",
                cameraPose2d.getTranslation().getX(),
                cameraPose2d.getTranslation().getY(),
                cameraPose2d.getRotation().getDegrees()));

            // if(currentPose.getX() < 2.5) { // test hack, throwing away values when moving
            // away from AT
            poseEstimator.addVisionMeasurement(cameraPose2d, cameraPose.timestampSeconds,
                VisionConstants.kVisionMeasurementStdDevs);
            // }
            // }
          }
        }
      }

      // Update pose estimator with drivetrain sensors
      poseEstimator.updateWithTime(Timer.getFPGATimestamp(), m_swerve.getHeading(), m_swerve.getPositions());
      field2d.setRobotPose(getCurrentPose());
    }

    // log pose estimations
    Pose2d currentPose = getCurrentPose();
     currentPose = getAutoPose();    // this is a hack - delete this line 
    poseLogger.add(new PoseEstimations(currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees()));
    if(Timer.getFPGATimestamp() - lastLogTime > logInterval) {
      poseLogger.flush();
      lastLogTime = Timer.getFPGATimestamp();
    }
  }

    /**
   * This method assumes should only be called when the robot is supposed to be
   * heading straight to a game piece. It will only return adjusted pose in the Y axis of the robot coordinate system.
   * This will not change the robot's underlying pose but rather just return 
   * an adjusted pose to the pathplanner while the useVisionCorrection flag is set. 
   * This is only intended to be used in auto.  To use, simply set EnableVisionCorrection in pathPlanner when you are
   * lined up and heading for a piece.  After you get the piece, set DisableVisionCorrection.
   * NOTE: we really should dynamically set the camera pitch based on the proximal and distal relative encoders. 
 
   */
  private Pose2d getAdjustedPose() {
    PhotonCamera camera = photonCamera3;
    if(!camera.isConnected()) {
      return poseEstimator.getEstimatedPosition();
    }

    Transform3d robotToCamera = this.cameraMap.get(VisionConstants.kCameraMount3Id).transform;
    var visionResult = camera.getLatestResult();

    if (visionResult.hasTargets()) {
      // double cameraAngleOutFromFloor = Units.degreesToRadians(10.0);
      double slantRange = robotToCamera.getZ()/Math.cos(cameraAngleOutFromFloor + Units.degreesToRadians(visionResult.getBestTarget().getPitch()));
      double lateralErrorCorrection = Units.inchesToMeters(-7.5) +robotToCamera.getZ()*Math.tan(Units.degreesToRadians(visionResult.getBestTarget().getYaw() ));

      SmartDashboard.putNumber("Slant Range to piece (inches)", Units.metersToInches(slantRange));
      SmartDashboard.putNumber("Lateral Correction (inches)", Units.metersToInches(lateralErrorCorrection));
      Pose2d adjustedPose = new Pose2d(
          getCurrentPose().getX() - (Math.sin(getCurrentPose().getRotation().getRadians()) * lateralErrorCorrection),
          getCurrentPose().getY() + (Math.cos(getCurrentPose().getRotation().getRadians()) * lateralErrorCorrection),
          getCurrentPose().getRotation());

      field2d.getObject("MyRobot Adjusted Pose").setPose(adjustedPose);
      gamePieceDetected = true;
      return adjustedPose;
    }
    
    field2d.getObject("MyRobot Adjusted Pose").setPose(poseEstimator.getEstimatedPosition());
    return poseEstimator.getEstimatedPosition();
  }

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f)", 
        Units.metersToInches(pose.getX()), 
        Units.metersToInches(pose.getY()));
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d getAutoPose() {
    if (useVisionCorrection) {
      return getAdjustedPose();
    }
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    m_swerve.zeroGyro();
    poseEstimator.resetPosition( m_swerve.getHeading(), m_swerve.getPositions(), newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    m_swerve.zeroGyro();
    poseEstimator.resetPosition(
       m_swerve.getHeading(), m_swerve.getPositions(), new Pose2d());
  }

  public void setVisionCorrection (boolean b, GamePiece pieceType ) {
    System.out.println("Setting Vision Correction " + b + ": Game Piece Detected:" + gamePieceDetected);
    cameraAngleOutFromFloor = pieceType == GamePiece.CONE? Units.degreesToRadians(VisionConstants.ConeAutoAngle): Units.degreesToRadians(VisionConstants.CubeAutoAngle);

    gamePieceDetected = false;
    this.useVisionCorrection = b;
  }

  public void enableAprilTags(boolean b) {
    aprilTagsIsEnabled = b;
  }


}