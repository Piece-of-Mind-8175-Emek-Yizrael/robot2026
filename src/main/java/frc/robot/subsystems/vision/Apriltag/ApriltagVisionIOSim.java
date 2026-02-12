package frc.robot.subsystems.vision.Apriltag;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

// FIXME: this is supposed to implement ApriltagVisionIO, not to extend ApriltagVisionIOReal
public class ApriltagVisionIOSim extends ApriltagVisionIOReal {

    private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

    public ApriltagVisionIOSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier){
        super(name, robotToCamera);
        this.poseSupplier = poseSupplier;
        if(visionSim == null){
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(aprilTagLayout);

        }
        var cameraProperties = new SimCameraProperties();
        cameraSim = new PhotonCameraSim(camera,cameraProperties);
        visionSim.addCamera(cameraSim, robotToCamera);
    }

    @Override
    public void updateInputs(ApriltagVisionIOInputs inputs){
        visionSim.update(poseSupplier.get());
        super.updateInputs(inputs);
    }

}
