package frc.robot.subsystems.vision.ObjectDetection.Lumen;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.lumenvision.lib.LumenDetectedObject;
import org.lumenvision.lib.LumenObjectDetectionResult;
import org.lumenvision.lib.LumenObjectDetector;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.ObjectDetection.Detection;
import frc.robot.subsystems.vision.ObjectDetection.ObjectDetectionVisionIO;

/** IO implementation for a fuel object-detection node on a LumenVision coprocessor. */
public class FuelIORealLumen implements ObjectDetectionVisionIO {
    final LumenObjectDetector detector;
    final String name;
    boolean toggle = true;

    Transform3d robotToCamera;

    /** @param name The object-detection node's name on the coprocessor. */
    public FuelIORealLumen(String name) {
        this.name = name;
        detector = new LumenObjectDetector(name);
    }

    @Override
    public void updateInputs(ObjectDetectionVisionIOInputs inputs) {
        inputs.pipelineName = name;
        inputs.connected = detector.isConnected();
        if (!toggle) return;

        // no new frame since the last loop: keep the detections already in the inputs
        List<LumenObjectDetectionResult> allUnreadResults = detector.getAllUnreadResults();
        if (allUnreadResults.isEmpty()) return;
        LumenObjectDetectionResult latestResult = allUnreadResults.get(allUnreadResults.size() - 1);

        List<Detection> detections = new ArrayList<>();
        for (LumenDetectedObject target : latestResult.getTargetsOfClass(VisionConstants.TargetType.FUEL.getClassId())) {
            // TODO: fill in the actual value for these variables
            double targetHeight = target.getBoxHeightPixels();
            double targetWidth = target.getBoxWidthPixels();
            if (targetWidth <= 0) continue;

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
