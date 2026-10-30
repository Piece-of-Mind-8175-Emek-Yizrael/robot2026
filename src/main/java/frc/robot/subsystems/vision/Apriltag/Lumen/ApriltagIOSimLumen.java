package frc.robot.subsystems.vision.Apriltag.Lumen;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import java.util.function.Supplier;

import org.lumenvision.photoncompat.sim.PhotonCameraSim;
import org.lumenvision.photoncompat.sim.SimCameraProperties;
import org.lumenvision.photoncompat.sim.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * Simulated AprilTag camera: lumenlib's simulator publishes the results a coprocessor would for the robot's simulated pose, and the real
 * IO reads them back from NetworkTables, so the whole path is exercised.
 */
public class ApriltagIOSimLumen extends ApriltagIORealLumen {
    private static VisionSystemSim visionSim;

    private final Supplier<Pose2d> poseSupplier;
    private final PhotonCameraSim cameraSim;

    public ApriltagIOSimLumen(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
        super(name, robotToCamera);
        this.poseSupplier = poseSupplier;
        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(aprilTagLayout);
        }
        var cameraProperties = new SimCameraProperties();
        cameraSim = new PhotonCameraSim(detector, cameraProperties);
        visionSim.addCamera(cameraSim, robotToCamera);
    }

    @Override
    public void setRobotToCamera(Transform3d robotToCamera) {
        super.setRobotToCamera(robotToCamera);
        visionSim.adjustCamera(cameraSim, robotToCamera);
    }

    @Override
    public void togglePipeline(boolean on) {
        super.togglePipeline(on);
        cameraSim.setEnabled(on);
    }

    @Override
    public void updateInputs(ApriltagVisionIOInputs inputs) {
        visionSim.update(poseSupplier.get());
        super.updateInputs(inputs);
    }
}
