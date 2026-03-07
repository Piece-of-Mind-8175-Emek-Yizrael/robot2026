package frc.robot.subsystems.vision.ObjectDetection;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.vision.VisionConstants;

import java.util.Optional;
import frc.robot.subsystems.vision.VisionConstants.TargetType;


public record Detection(
        TargetType targetType,
        double targetWidth,
        double targetHeight,
        double cameraRelativeTargetRotation,
        Optional<Translation2d> cameraRelativeTargetTranslation
){}
