package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
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

  // Simulation objects
  private VisionSystemSim visionSim;
  private final List<PhotonCameraSim> cameraSims = new ArrayList<>();

  public VisionSubsystem() {
    if (RobotBase.isSimulation()) {
      visionSim = new VisionSystemSim("main");
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
          new Translation3d(-0.28734, 0.26194, 0.1782), new Rotation3d(0, -0.436332, 1.570796)),
      new Transform3d( // Back right camera (limelight4)
          new Translation3d(-0.2259, -0.22538, 0.1797), new Rotation3d(0, -0.436332, 3.14159))
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
  }

  @Override
  public void periodic() {
    if (RobotBase.isSimulation()) {
      // In a real project, you'd pass your actual Drive Pose here
      visionSim.update(new Pose2d());
    }
    visionEstimates.clear();
    // Process results from all cameras
    for (int i = 0; i < photonPoseEstimators.size(); i++) {
      PhotonCamera cam = cameras.get(i);
      List<PhotonPipelineResult> results = cam.getAllUnreadResults();
      for (PhotonPipelineResult result : results) {
        if (result.hasTargets()) {
          visionEstimates.add(photonPoseEstimators.get(i).estimateCoprocMultiTagPose(result));
        }
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

  public Matrix<N3, N1> getEstimationStdDevs(EstimatedRobotPose estimate) {
    double estStdDevX = 0.5;
    double estStdDevY = 0.5;
    double estStdDevTheta = 999; // Apparently PV is bad at rotation
    // Gyro is almost always better than vision for rotation

    final int numTags = estimate.targetsUsed.size();
    double avgDistance = 0;

    for (PhotonTrackedTarget target : estimate.targetsUsed) {
      avgDistance += target.getBestCameraToTarget().getTranslation().getNorm();
    }
    avgDistance /= numTags;

    if (numTags > 1) { // Multiple tags, trust more
      estStdDevX = 0.1;
      estStdDevY = 0.1;
    } else if (avgDistance > 4.0) { // If far, trust less
      estStdDevX = 1.0;
      estStdDevY = 1.0;
    }

    // Scale standard deviation by distance squared (common FRC practice)
    final double distanceMultiplier = Math.max(1, Math.pow(avgDistance, 2) / 16.0);

    return VecBuilder.fill(
        estStdDevX * distanceMultiplier, estStdDevY * distanceMultiplier, estStdDevTheta);
  }
}
