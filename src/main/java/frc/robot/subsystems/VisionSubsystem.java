package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
  // Define camera names as they appear in the PhotonVision dashboard
  private final String[] cameraNames = {"Limelight3-BackLeftSwerve", "Limelight4-BackRightSwerve"};
  private final List<PhotonCamera> cameras = new ArrayList<>();

  // Photon pose estimators
  private final List<PhotonPoseEstimator> photonPoseEstimators = new ArrayList<>();

  public static final AprilTagFieldLayout tagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  List<Optional<EstimatedRobotPose>> visionEstimates = new ArrayList<>();

  // Pose3ds for atvantageScope
  List<Pose3d> visionPose3ds = new ArrayList<>();

  StructArrayPublisher<Pose3d> arrayPublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("VisionPoseArray", Pose3d.struct)
          .publish();

  // Field2d for pose visualization
  private List<Field2d> field2ds = new ArrayList<>();

  Supplier<Pose2d> currentRobotPose;

  // Simulation objects
  private VisionSystemSim visionSim;
  private final List<PhotonCameraSim> cameraSims = new ArrayList<>();

  public VisionSubsystem(Supplier<Pose2d> currentRobotPose) {
    this.currentRobotPose = currentRobotPose;
    if (RobotBase.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(tagLayout);
    }

    // Define shared properties for the cameras
    SimCameraProperties cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
    cameraProp.setCalibError(0.25, 0.08);
    cameraProp.setFPS(20);
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);

    // Define physical mounting positions (Robot-to-Camera transforms)
    Transform3d[]
        robotToCamTransforms = { // These values use the pigeon as center, measurments in meters
      // from CAD
      new Transform3d( // Back left camera (limelight3)
          new Translation3d(-0.28734, -0.22538, 0.1782), new Rotation3d(0, -0.436332, 1.570796)),
      new Transform3d( // Back right camera (limelight4)
          new Translation3d(-0.2259, 0.26194, 0.1797), new Rotation3d(0, -0.436332, 3.14159))
    };

    for (int i = 0; i < cameraNames.length; i++) {
      PhotonCamera cam = new PhotonCamera(cameraNames[i]);
      cameras.add(cam);
      PhotonPoseEstimator poseEstimator =
          new PhotonPoseEstimator(tagLayout, robotToCamTransforms[i]);
      photonPoseEstimators.add(poseEstimator);

      if (RobotBase.isSimulation()) {
        PhotonCameraSim camSim = new PhotonCameraSim(cam, cameraProp);
        visionSim.addCamera(camSim, robotToCamTransforms[i]);
        cameraSims.add(camSim);
      }
    }
    for (int i = 0; i < cameraNames.length; i++) {
      SmartDashboard.putData("VisionField" + i, field2ds.get(i));
    }
  }

  @Override
  public void periodic() {
    if (RobotBase.isSimulation()) {
      // In a real project, you'd pass your actual Drive Pose here
      visionSim.update(currentRobotPose.get());
    }
    visionEstimates.clear();
    visionPose3ds.clear();
    // Process results from all cameras
    for (int i = 0; i < photonPoseEstimators.size(); i++) {
      PhotonCamera cam = cameras.get(i);
      List<PhotonPipelineResult> results = cam.getAllUnreadResults();
      for (PhotonPipelineResult result : results) {
        if (result.hasTargets()) {
          visionEstimates.add(photonPoseEstimators.get(i).estimateCoprocMultiTagPose(result));
          visionPose3ds.add(visionEstimates.get(i).get().estimatedPose);
        }
      }
    }
    if (!visionPose3ds.isEmpty()) {
      arrayPublisher.set(new Pose3d[] {visionPose3ds.get(0), visionPose3ds.get(1)});
      for (int i = 0; i < cameraNames.length; i++) {
        field2ds.get(i).setRobotPose(visionPose3ds.get(i).toPose2d());
      }
    }
  }

  public List<Optional<EstimatedRobotPose>> getPoseEstimates() {
    return visionEstimates;
  }

  /**
   * Searches all cameras for the closest AprilTag.
   *
   * @return An Optional containing the closest target found by any camera.
   */
  public Optional<PhotonTrackedTarget> getClosestTag() {
    return cameras.stream()
        .map(cam -> cam.getLatestResult())
        .filter(result -> result.hasTargets())
        .flatMap(result -> result.getTargets().stream())
        .filter(t -> t.getFiducialId() > 0)
        .min(Comparator.comparingDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm()));
  }
}
