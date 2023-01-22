package frc.robot.subsystems;

import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

  // Physical location of the camera on the robot, relative to the center of the
  // robot.
  private static final Transform3d CAMERA_TO_ROBOT_1 = 
      new Transform3d(new Translation3d(Units.inchesToMeters(15.5), Units.inchesToMeters(-5.0),0.5), new Rotation3d(0.0,0.0,0.0));
  private static final Transform3d CAMERA_TO_ROBOT_2 = 
      new Transform3d(new Translation3d(Units.inchesToMeters(15.5), Units.inchesToMeters(-5.0),0.5), new Rotation3d(Units.degreesToRadians(270),0.0,Units.degreesToRadians(180)));

  // Ordered list of target poses by ID (WPILib is adding some functionality for
  // this)
  private static final List<Pose3d> targetPoses = Collections.unmodifiableList(List.of(
      new Pose3d(7.642, 4.139,1.515, new Rotation3d(0.0, 0.0,Units.degreesToRadians(180))),
      new Pose3d(8.162, 3.619,1.515, new Rotation3d(0.0, 0.0,Units.degreesToRadians(270))),
      new Pose3d(8.502, 4.139,1.515, new Rotation3d(0.0, 0.0,Units.degreesToRadians(0))),
      new Pose3d(7.982, 4.659,1.515, new Rotation3d(0.0, 0.0,Units.degreesToRadians(90))),
      new Pose3d(13.540, 2.281,1.410, new Rotation3d(0.0, 0.0,Units.degreesToRadians(180))),
      new Pose3d(0.0, 1.850,1.075,new Rotation3d(0.0, 0.0,Units.degreesToRadians(0)))));
  
  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.
  private static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(180)); // was 5
  private static final Matrix<N1, N1> localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.01));
  private static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(180)); // was 5
  private final SwerveDrivePoseEstimator poseEstimator;
  private HashMap<String, CameraTransform> cameraMap = new HashMap<String, CameraTransform>();
  public static final String CAM1 = "Global_Shutter_Camera";
  public static final String CAM2 = "Microsoft_LifeCam_HD-3000";
  private final PhotonCamera photonCamera1 = new PhotonCamera(CAM1);
  private final PhotonCamera photonCamera2 = new PhotonCamera(CAM2);


  private final Field2d field2d = new Field2d();

  public PoseEstimatorSubsystem( Swerve m_swerve) {
    this.cameraMap.put(CAM1, new CameraTransform(photonCamera1, CAMERA_TO_ROBOT_1));
    this.cameraMap.put(CAM2, new CameraTransform(photonCamera2, CAMERA_TO_ROBOT_2));
    this.m_swerve = m_swerve;

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");



    poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, m_swerve.getYaw(), m_swerve.getPositions(), new Pose2d());
    
    
    tab.addString("Pose (X, Y)", this::getFomattedPose).withPosition(0, 4);
    tab.addNumber("Pose Degrees", () -> getCurrentPose().getRotation().getDegrees()).withPosition(1, 4);
    tab.add(field2d);
  }

  @Override
  public void periodic() {
    for(String cameraName : this.cameraMap.keySet()){
      // Update pose estimator with visible targets

      var res = this.cameraMap.get(cameraName).camera.getLatestResult();

      
      if (res.hasTargets()) {
    
        double imageCaptureTime = Timer.getFPGATimestamp() - (res.getLatencyMillis() / 1000d);
  
        for (PhotonTrackedTarget target : res.getTargets()) {
      
          var fiducialId = target.getFiducialId();

          if ( target.getPoseAmbiguity() < 0.2 && fiducialId >= 0 && fiducialId < targetPoses.size()) {
          
            var targetPose = targetPoses.get(fiducialId);

            Transform3d camToTargetBest = target.getBestCameraToTarget();
            Transform3d camToTargetAlt = target.getAlternateCameraToTarget();
            

            Pose3d camPoseBest = targetPose.transformBy(camToTargetBest.inverse());
            Pose3d camPoseAlt = targetPose.transformBy(camToTargetAlt.inverse());

            var visionMeasurementBest = camPoseBest.transformBy(this.cameraMap.get(cameraName).transform).toPose2d();
            var visionMeasurementAlt = camPoseAlt.transformBy(this.cameraMap.get(cameraName).transform).toPose2d();

            var distanceBest = visionMeasurementBest.getTranslation().getDistance(getCurrentPose().getTranslation());
            var distanceAlt = visionMeasurementAlt.getTranslation().getDistance(getCurrentPose().getTranslation());
            
            var angleBest = visionMeasurementBest.getRotation().getRadians() - getCurrentPose().getRotation().getRadians();
            var angleAlt = visionMeasurementAlt.getRotation().getRadians() - getCurrentPose().getRotation().getRadians();


           // uncomment the line below and comment out the line below that one to use distance
           // Pose2d visionMeasurement = (distanceBest > distanceAlt) ?  visionMeasurementAlt.toPose2d():  visionMeasurementBest.toPose2d();
           Pose2d visionMeasurement = (angleBest > angleAlt) ?  visionMeasurementAlt:  visionMeasurementBest;

            field2d.getObject("MyRobotBest" + fiducialId + cameraName).setPose(visionMeasurementBest);
            field2d.getObject("MyRobotAlt" + fiducialId + cameraName).setPose(visionMeasurementAlt);

            SmartDashboard.putString("Vision pose", String.format("(%.2f, %.2f) %.2f",
              visionMeasurement.getTranslation().getX(),
              visionMeasurement.getTranslation().getY(),
              visionMeasurement.getRotation().getDegrees()));
            poseEstimator.addVisionMeasurement(visionMeasurement, imageCaptureTime);
          }
        }
            // Update pose estimator with drivetrain sensors
      }
      poseEstimator.updateWithTime(Timer.getFPGATimestamp(), m_swerve.getYaw(), m_swerve.getPositions());

      field2d.setRobotPose(getCurrentPose());

    }
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

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    m_swerve.zeroGyro();
    poseEstimator.resetPosition( m_swerve.getYaw(), m_swerve.getPositions(), newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    m_swerve.zeroGyro();
    poseEstimator.resetPosition(
       m_swerve.getYaw(), m_swerve.getPositions(), new Pose2d());
  }

}