package frc.robot.subsystems.vision.ObjectDetection;

import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionVisionIO {
    @AutoLog
    public static class ObjectDetectionVisionIOInputs {
        public String pipelineName = "";
        public boolean connected = false;
        public Detection[] detections = new Detection[0];
    }

    public void updateInputs(ObjectDetectionVisionIOInputs inputs);

    public String getPipelineName();

    public void setRobotToCamera(Transform3d robotToCamera);

    public void togglePipeline(boolean on);

}
