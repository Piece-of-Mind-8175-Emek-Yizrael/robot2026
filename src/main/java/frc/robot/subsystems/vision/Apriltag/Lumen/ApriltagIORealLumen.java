package frc.robot.subsystems.vision.Apriltag.Lumen;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.lumenvision.lib.LumenDetector;
import org.lumenvision.lib.LumenMultiTagResult;
import org.lumenvision.lib.LumenPipelineResult;
import org.lumenvision.lib.LumenPoseEstimator;
import org.lumenvision.lib.LumenRobotPose;
import org.lumenvision.lib.LumenTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.Apriltag.ApriltagVisionIO;

/** IO implementation for an AprilTag detector node on a LumenVision coprocessor. */
public class ApriltagIORealLumen implements ApriltagVisionIO {
    protected final LumenDetector detector;
    // composes the coprocessor's multi-tag solve (the camera's field pose) with the camera mount
    private final LumenPoseEstimator poseEstimator;
    protected Transform3d robotToCamera;
    private boolean enabled = true;

    /**
     * @param name          The detector node's name on the coprocessor.
     * @param robotToCamera The camera's position relative to the robot.
     */
    public ApriltagIORealLumen(String name, Transform3d robotToCamera) {
        detector = new LumenDetector(name);
        poseEstimator = new LumenPoseEstimator(aprilTagLayout, robotToCamera);
        this.robotToCamera = robotToCamera;
    }

    @Override
    public void updateInputs(ApriltagVisionIOInputs inputs) {
        inputs.connected = detector.isConnected();
        inputs.pipelineName = detector.getName();

        Set<Integer> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new ArrayList<>();

        // read every loop, so results queued while the detector was off are dropped instead of used once it is back on
        List<LumenPipelineResult> results = detector.getAllUnreadResults();
        if (enabled) {
            for (LumenPipelineResult result : results) {
                // Update latest target observation
                Optional<LumenTrackedTarget> best = result.getBestTarget();
                if (best.isPresent()) {
                    inputs.latestTargetObservation = new TargetObservation(
                            Rotation2d.fromDegrees(best.get().getYaw()),
                            Rotation2d.fromDegrees(best.get().getPitch()));
                    inputs.cameraToBestTarget = best.get().getBestCameraToTarget();
                } else {
                    inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
                    inputs.cameraToBestTarget = null;
                }

                // Add pose observation
                Optional<LumenRobotPose> multiTagPose = poseEstimator.estimateMultiTag(result);
                if (multiTagPose.isPresent()) { // Multitag result
                    LumenMultiTagResult multiTag = result.getMultiTagResult().get();
                    Logger.recordOutput(detector.getName() + "Target Yaw", best.map(LumenTrackedTarget::getYaw).orElse(0.0));

                    // Calculate average tag distance
                    double totalTagDistance = 0.0;
                    for (LumenTrackedTarget target : result.getTargets()) {
                        totalTagDistance += target.getBestCameraToTarget().getTranslation().getNorm();
                    }

                    // Add tag IDs
                    tagIds.addAll(multiTag.getFiducialIds());

                    // Add observation
                    poseObservations.add(
                            new PoseObservation(
                                    result.getTimestampSeconds(), // Timestamp (the frame's capture time)
                                    multiTagPose.get().getPose(), // 3D pose estimate
                                    0.0, // Ambiguity: a multi-tag solve has a single answer
                                    multiTag.getTagCount(), // Tag count
                                    totalTagDistance / result.getTargets().size(), // Average tag distance
                                    PoseObservationType.LUMENVISION)); // Observation type

                } else if (result.hasTargets()) { // Single tag result
                    LumenTrackedTarget target = result.getTargets().get(0);

                    // A detector without a camera calibration solves no pose (identity transform, ambiguity -1)
                    if (target.getPoseAmbiguity() < 0) continue;

                    // Calculate robot pose
                    var tagPose = aprilTagLayout.getTagPose(target.getFiducialId());
                    if (tagPose.isPresent()) {
                        Transform3d fieldToTarget = new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
                        Transform3d cameraToTarget = target.getBestCameraToTarget();
                        Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                        // Add tag ID
                        tagIds.add(target.getFiducialId());

                        // Add observation
                        poseObservations.add(
                                new PoseObservation(
                                        result.getTimestampSeconds(), // Timestamp (the frame's capture time)
                                        robotPose, // 3D pose estimate
                                        target.getPoseAmbiguity(), // Ambiguity
                                        1, // Tag count
                                        cameraToTarget.getTranslation().getNorm(), // Average tag distance
                                        PoseObservationType.LUMENVISION)); // Observation type
                    }
                }
            }
        }

        // Save pose observations to inputs object
        inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);

        // Save tag IDs to inputs objects
        inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
    }

    @Override
    public String getPipelineName() {
        return detector.getName();
    }

    @Override
    public void setRobotToCamera(Transform3d robotToCamera) {
        this.robotToCamera = robotToCamera;
        poseEstimator.setRobotToCameraTransform(robotToCamera);
    }

    @Override
    public void togglePipeline(boolean on) {
        enabled = on;
        // stops the detector node on the coprocessor too, so it spends nothing while the camera cannot be used
        detector.setEnabled(on);
    }
}
