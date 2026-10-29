package frc.robot.subsystems.vision.Apriltag.Lumen;

import org.lumenvision.lib.*;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.vision.Apriltag.ApriltagVisionIO;

public class ApriltagIORealLumen implements ApriltagVisionIO {
    LumenDetector m_LumenDetector;
    Transform3d m_RobotToCamera;

    public ApriltagIORealLumen(String name, Transform3d robotToCamera) {
        // Initialize Lumen coprocessor and camera here
        m_LumenDetector = new LumenDetector(name);
        m_RobotToCamera = robotToCamera;
    }

    @Override
    public void updateInputs(ApriltagVisionIOInputs inputs) {
        // Update inputs from Lumen detector
        inputs.connected = m_LumenDetector.isConnected();
        inputs.pipelineName = m_LumenDetector.getName();
        inputs.latestTargetObservation = null; // as this is for pose estimation, we might not have a simple target observation
        
        if (m_LumenDetector.getMultiTagResult().isPresent()) {
            LumenMultiTagResult result = m_LumenDetector.getMultiTagResult().get();
            inputs.tagIds = result.getFiducialIds().stream().mapToInt(Integer::intValue).toArray();


            inputs.poseObservations = result.getPoseObservations();
            inputs.cameraToBestTarget = result.getCameraToBestTarget();
        }
        // TODO: update these
        // inputs.poseObservations = m_LumenDetector.getPoseObservations();
        // inputs.tagIds = m_LumenDetector.getTagIds();
        // inputs.cameraToBestTarget = m_LumenDetector.getCameraToBestTarget();
    }

    @Override
    public String getPipelineName() {
        return m_LumenDetector.getName();
    }

    @Override
    public void setRobotToCamera(Transform3d robotToCamera) {
        m_RobotToCamera = robotToCamera;
    }

    @Override
    public void togglePipeline(boolean on) {
        m_LumenDetector.setEnabled(on);
    }
}
