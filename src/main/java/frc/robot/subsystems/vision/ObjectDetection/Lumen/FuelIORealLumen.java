package frc.robot.subsystems.vision.ObjectDetection.Lumen;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.lumenvision.lib.LumenDetector;
import org.lumenvision.lib.LumenPipelineResult;
import org.lumenvision.lib.LumenTargetCorner;
import org.lumenvision.lib.LumenTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.ObjectDetection.Detection;
import frc.robot.subsystems.vision.ObjectDetection.ObjectDetectionVisionIO;

/** IO implementation for a fuel object-detection node on a LumenVision coprocessor. */
public class FuelIORealLumen implements ObjectDetectionVisionIO {
    final LumenDetector detector;
    final String name;
    boolean toggle = true;

    Transform3d robotToCamera;

    /** @param name The object-detection node's name on the coprocessor. */
    public FuelIORealLumen(String name) {
        this.name = name;
        detector = new LumenDetector(name);
    }

    @Override
    public void updateInputs(ObjectDetectionVisionIOInputs inputs) {
        inputs.pipelineName = name;
        inputs.connected = detector.isConnected();
        if (!toggle) return;

        // no new frame since the last loop: keep the detections already in the inputs
        List<LumenPipelineResult> allUnreadResults = detector.getAllUnreadResults();
        if (allUnreadResults.isEmpty()) return;
        LumenPipelineResult latestResult = allUnreadResults.get(allUnreadResults.size() - 1);

        List<Detection> detections = new ArrayList<>();
        for (LumenTrackedTarget target : latestResult.getTargets()) {
            if (target.getDetectedObjectClassID() != VisionConstants.TargetType.FUEL.getClassId()) continue;

            // an object's corners are its bounding box
            List<LumenTargetCorner> corners = sortCorners(target.getDetectedCorners());
            if (corners.size() < 4) continue;
            // TODO: fill in the actual value for these variables
            // the corners run top left, top right, bottom right, bottom left with image y pointing down, so the differences are negative
            double targetHeight = Math.abs(corners.get(1).y - corners.get(2).y);
            double targetWidth = Math.abs(corners.get(0).x - corners.get(1).x);

            double distance = getDistanceToFuelMeters(targetWidth);
            double yawRad = Math.toRadians(target.getYaw());
            Translation2d translationToFuel = new Translation2d(distance * Math.cos(yawRad), distance * Math.sin(yawRad));

            detections.add(new Detection(
                    VisionConstants.TargetType.FUEL,
                    targetWidth,
                    targetHeight,
                    target.getYaw(),
                    Optional.of(translationToFuel)));
        }

        inputs.detections = detections.toArray(new Detection[0]);
    }

    @Override
    public String getPipelineName() {
        return name;
    }

    @Override
    public void setRobotToCamera(Transform3d robotToCamera) {
        this.robotToCamera = robotToCamera;
    }

    @Override
    public void togglePipeline(boolean on) {
        toggle = on;
    }

    // sorts corners by this order: top left, top right, bottom right, bottom left
    private List<LumenTargetCorner> sortCorners(List<LumenTargetCorner> orgCorners) {
        if (orgCorners == null || orgCorners.isEmpty()) {
            return new ArrayList<>();
        }

        double cx = 0.0;
        double cy = 0.0;
        for (LumenTargetCorner c : orgCorners) {
            cx += c.x;
            cy += c.y;
        }
        cx /= orgCorners.size();
        cy /= orgCorners.size();

        List<LumenTargetCorner> sorted = new ArrayList<>(orgCorners);
        double finalCy = cy;
        double finalCx = cx;
        sorted.sort(java.util.Comparator.comparingDouble(c -> Math.atan2(c.y - finalCy, c.x - finalCx)));

        return sorted;
    }

    // calculated by dist = (focal length * real width) / perceived width
    // the coprocessor publishes the camera matrix of an object-detection node once its camera is calibrated at the running resolution
    private double getDistanceToFuelMeters(double targetWidthInPixels) {
        if (detector.getCameraMatrix().isPresent())
            return (detector.getCameraMatrix().get().get(0, 0)
                    * VisionConstants.ObjectDetectionConstants.FUEL_DIAMETER_METERS)
                    / targetWidthInPixels;
        Logger.recordOutput("vision/errors", "Camera matrix not present");
        return 0;
    }
}
