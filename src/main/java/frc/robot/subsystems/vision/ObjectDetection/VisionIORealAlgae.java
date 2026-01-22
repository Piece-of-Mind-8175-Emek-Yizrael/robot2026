package frc.robot.subsystems.vision.ObjectDetection;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.vision.VisionConstants;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class VisionIORealAlgae implements ObjectDetectionVisionIO {
    final PhotonCamera camera;
    final String name;

    public VisionIORealAlgae(String name) {
        this.name = name;
        camera = new PhotonCamera(name);
    }

    @Override
    public void updateInputs(ObjectDetectionVisionIOInputs inputs) {
        List<PhotonPipelineResult> allUnreadResults = camera.getAllUnreadResults();
        PhotonPipelineResult latestResult = allUnreadResults.get(allUnreadResults.size() - 1);

        Detection[] detections = new Detection[latestResult.getTargets().size()];
        for(int i = 0; i < detections.length; i++) {
            PhotonTrackedTarget target = latestResult.getTargets().get(i);

            List<TargetCorner> corners = sortCorners(target.getDetectedCorners());
            // TODO: fill in the actual value for these variables
            double targetHeight = corners.get(1).y - corners.get(2).y;
            double targetWidth = corners.get(0).x - corners.get(1).x;

            if(target.getDetectedObjectClassID() == VisionConstants.TargetType.ALGAE.getClassId()) {

                double distance = getDistanceToAlgaeMeters(targetWidth);
                double yawRad = Math.toRadians(target.getYaw());
                Translation2d translationToAlgae = new Translation2d(distance * Math.cos(yawRad), distance * Math.sin(yawRad));

                detections[i] = new Detection(
                        VisionConstants.TargetType.ALGAE,
                        targetWidth,
                        targetHeight,
                        target.getYaw(),
                        Optional.of(translationToAlgae)
                );
            } else if(target.getDetectedObjectClassID() == VisionConstants.TargetType.CORAL.getClassId()) {
                detections[i] = new Detection(
                        VisionConstants.TargetType.CORAL,
                        targetWidth,
                        targetHeight,
                        target.getYaw(),
                        Optional.empty()
                );
            }
        }

        inputs.pipelineName = name;
        inputs.connected = camera.isConnected();
        inputs.detections = detections;
    }

    // sorts corners by this order: top left, top right, bottom right, bottom left
    private List<TargetCorner> sortCorners(List<TargetCorner> orgCorners) {
        if (orgCorners == null || orgCorners.isEmpty()) {
            return new ArrayList<>();
        }

        double cx = 0.0;
        double cy = 0.0;
        for (TargetCorner c : orgCorners) {
            cx += c.x;
            cy += c.y;
        }
        cx /= orgCorners.size();
        cy /= orgCorners.size();

        List<TargetCorner> sorted = new ArrayList<>(orgCorners);
        double finalCy = cy;
        double finalCx = cx;
        sorted.sort(java.util.Comparator.comparingDouble(c -> Math.atan2(c.y - finalCy, c.x - finalCx)));

        return sorted;
    }

    // calculated by dist = (focal length * real width) / perceived width
    private double getDistanceToAlgaeMeters(double targetWidthInPixels) {
        if(camera.getCameraMatrix().isPresent())
            return (camera.getCameraMatrix().get().get(0,0)
                    * VisionConstants.ObjectDetectionConstants.ALGAE_DIAMETER_METERS)
                    / targetWidthInPixels;
        Logger.recordOutput("vision/errors", "Camera matrix not present");
        return 0;
    }
}
