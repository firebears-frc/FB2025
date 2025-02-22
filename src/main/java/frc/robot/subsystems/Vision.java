package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.VisionData;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private final PhotonCamera photonCamera;
  private final PhotonPoseEstimator poseEstimator;
  private final Consumer<VisionData> consumer;
  private final String name;

  public Vision(Consumer<VisionData> consumer, Transform3d cameraOffSet, String name)
      throws IOException {
    photonCamera = new PhotonCamera(name);
    poseEstimator =
        new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            cameraOffSet);
    this.consumer = consumer;
    this.name = name;
  }

  private Matrix<N3, N1> getStdDevs(List<PhotonTrackedTarget> targets, Pose3d estimatedPose) {
    int targetCount = 0;
    double distance = 0;
    for (var target : targets) {
      var tagPose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
      if (tagPose.isEmpty()) {
        continue;
      }

      targetCount++;
      distance += tagPose.get().getTranslation().getDistance(estimatedPose.getTranslation());
    }

    // If we have no targets, exit early
    if (targetCount == 0) {
      return null;
    }

    // Convert to average distance
    distance /= targetCount;

    // Decrease std devs if multiple targets are visible
    Matrix<N3, N1> stdDevs;
    if (targetCount > 1) {
      stdDevs = Constants.VisionConstants.MULTI_TAG_STD_DEVS;
    } else {
      stdDevs = Constants.VisionConstants.SINGLE_TAG_STD_DEVS;
    }

    // Increase std devs based on (average) distance
    if (targetCount == 1.0 && distance > 4.0) {
      stdDevs = null;
    } else {
      stdDevs = stdDevs.times(1 + (distance * distance / 30.0));
    }

    return stdDevs;
  }

  @Override
  public void periodic() {
    boolean connected = photonCamera.isConnected();
    Logger.recordOutput("Vision/" + name + "/Connected", connected);
    if (!connected) return;

    PhotonPipelineResult pipelineResult = photonCamera.getLatestResult();
    boolean hasTargets = pipelineResult.hasTargets();
    Logger.recordOutput("Vision/" + name + "/HasTargets", hasTargets);
    if (!hasTargets) return;

    List<PhotonTrackedTarget> badTargets = new ArrayList<>();
    for (PhotonTrackedTarget target : pipelineResult.targets) {
      if (target.getPoseAmbiguity() > .1 || target.getPoseAmbiguity() == 0) {
        badTargets.add(target);
      }
    }

    pipelineResult.targets.removeAll(badTargets);
    Logger.recordOutput("Vision/" + name + "/badTargets", badTargets.size());

    Optional<EstimatedRobotPose> poseResult = poseEstimator.update(pipelineResult);
    boolean posePresent = poseResult.isPresent();
    Logger.recordOutput("Vision/" + name + "/HasPose", posePresent);
    if (!posePresent) return;

    EstimatedRobotPose estimatedPose = poseResult.get();
    Logger.recordOutput("Vision/" + name + "/Pose", estimatedPose.estimatedPose);
    Logger.recordOutput("Vision/" + name + "/Timestamp", estimatedPose.timestampSeconds);
    Logger.recordOutput("Vision/" + name + "/Targets", estimatedPose.targetsUsed.size());
    Logger.recordOutput("Vision/" + name + "/Strategy", estimatedPose.strategy);
    Logger.recordOutput(
        "Vision/" + name + "/ambiguity", estimatedPose.targetsUsed.get(0).getPoseAmbiguity());

    Matrix<N3, N1> stdDevs = getStdDevs(pipelineResult.targets, estimatedPose.estimatedPose);
    if (stdDevs == null) {
      return;
    }

    consumer.accept(
        new VisionData(estimatedPose.estimatedPose, estimatedPose.timestampSeconds, stdDevs));
  }
}
