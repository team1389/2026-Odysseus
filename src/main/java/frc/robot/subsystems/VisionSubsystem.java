package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
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

  private boolean isTrusted(EstimatedRobotPose estimate) {
    int tagCount = estimate.targetsUsed.size();

    if (tagCount == 0) {
      return false;
    }
    // single tag measurement
    if (tagCount == 1) {
      double ambiguity = estimate.targetsUsed.get(0).getPoseAmbiguity();
      if (ambiguity > 0.2 || ambiguity < 0) {
        return false;
      }
    }
    // measurements for when tags are too far
    double averageDistance =
        estimate.targetsUsed.stream()
            .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(Double.POSITIVE_INFINITY);
    if (averageDistance > 6.0) {
      return false;
    }
    return true;
  }

  private Matrix<N3, N1> computeSTDevs(EstimatedRobotPose estimate) {
    int tagCount = estimate.targetsUsed.size();
    double averageDistance =
        estimate.targetsUsed.stream()
            .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(Double.POSITIVE_INFINITY);
    double baseXYStdDev = tagCount >= 2 ? 0.05 : 0.15;
    double distanceScale = 1.0 + (averageDistance * averageDistance * 0.05);
    return VecBuilder.fill(baseXYStdDev * distanceScale, baseXYStdDev * distanceScale, 4.0);
  }

  List<Optional<EstimatedRobotPose>> visionEstimates = new ArrayList<>();

  // Simulation objects
  private VisionSystemSim visionSim;
  private final List<PhotonCameraSim> cameraSims = new ArrayList<>();
  private final CommandSwerveDrivetrain drivetrain;

  public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
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
        if (!result.hasTargets()) continue;

        Optional<EstimatedRobotPose> estimate =
            photonPoseEstimators.get(i).estimateCoprocMultiTagPose(result);
        if (estimate.isEmpty()) continue;

        EstimatedRobotPose poseEstimate = estimate.get();
        if (!isTrusted(poseEstimate)) {
          SignalLogger.writeBoolean("Vision/Accepted", false);
          continue;
        }

        Matrix<N3, N1> stdDevs = computeSTDevs(poseEstimate);

        SignalLogger.writeBoolean("Vision/Accepted", true);
        SignalLogger.writeDouble("Vision/TagCount", poseEstimate.targetsUsed.size());
        SignalLogger.writeDouble("Vision/StdDevX", stdDevs.get(0, 0));
        SignalLogger.writeDouble("Vision/stdDevY", stdDevs.get(1, 0));
        SignalLogger.writeDouble("Vision/StdDevTheta", stdDevs.get(2, 0));
        SignalLogger.writeDouble("Vision/PoseX", poseEstimate.estimatedPose.getX());
        SignalLogger.writeDouble("Vision/PoseY", poseEstimate.estimatedPose.getY());

        drivetrain.addVisionMeasurement(
            poseEstimate.estimatedPose.toPose2d(), poseEstimate.timestampSeconds, stdDevs);

        visionEstimates.add(Optional.of(poseEstimate));
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
