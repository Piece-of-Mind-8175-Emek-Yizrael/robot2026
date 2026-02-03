
// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.Apriltag.ApriltagVisionIO;
import frc.robot.subsystems.vision.Apriltag.ApriltagVisionIO.PoseObservationType;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;

import frc.robot.subsystems.vision.Apriltag.ApriltagVisionIOInputsAutoLogged;
import frc.robot.subsystems.vision.ObjectDetection.Detection;
import frc.robot.subsystems.vision.ObjectDetection.ObjectDetectionVisionIO;
import frc.robot.subsystems.vision.ObjectDetection.ObjectDetectionVisionIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
    private final VisionConsumer consumer;
    private final ApriltagVisionIO[] apriltagVisionIO;
    private final ApriltagVisionIOInputsAutoLogged[] apriltagInputs;
    private final Alert[] disconnectedAlerts;

    private final ObjectDetectionVisionIO[] objectDetectionIO;
    private final ObjectDetectionVisionIOInputsAutoLogged[] objectDetectionInputs;
    private final List<Detection> detections = new ArrayList<>();
    private volatile boolean updatingDetections = false;

    public VisionSubsystem(VisionConsumer consumer, ApriltagVisionIO[] apriltagVisionIO,
            ObjectDetectionVisionIO[] objectDetectionIO) {
        this.consumer = consumer;
        this.apriltagVisionIO = apriltagVisionIO;
        this.objectDetectionIO = objectDetectionIO;

        // Initialize inputs
        this.apriltagInputs = new ApriltagVisionIOInputsAutoLogged[apriltagVisionIO.length];
        for (int i = 0; i < apriltagInputs.length; i++) {
            apriltagInputs[i] = new ApriltagVisionIOInputsAutoLogged();
        }

        if (objectDetectionIO != null) {
            this.objectDetectionInputs = new ObjectDetectionVisionIOInputsAutoLogged[objectDetectionIO.length];
            for (int i = 0; i < objectDetectionInputs.length; i++) {
                objectDetectionInputs[i] = new ObjectDetectionVisionIOInputsAutoLogged();
            }
        } else {
            this.objectDetectionInputs = new ObjectDetectionVisionIOInputsAutoLogged[0];
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[apriltagVisionIO.length];
        for (int i = 0; i < apriltagInputs.length; i++) {
            disconnectedAlerts[i] = new Alert(
                    "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
        }
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing
     * with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex) {
        return apriltagInputs[cameraIndex].latestTargetObservation.tx();
    }

    public ArrayList<Pair<Integer, Transform3d>> getReefTagsPositions() {
        return null;
    }

    public ArrayList<Pair<Integer, Transform3d>> getCoralStationTagsPositions() {
        return null;
    }

    public Transform3d getBestTarget(int camera) {
        return apriltagInputs[camera].cameraToBestTarget;
    }

    @Override
    public void periodic() {
        for (int i = 0; i < apriltagVisionIO.length; i++) {
            apriltagVisionIO[i].updateInputs(apriltagInputs[i]);
            Logger.processInputs("Vision/ApriltagCamera" + Integer.toString(i), apriltagInputs[i]);
        }

        if (objectDetectionIO != null) {
            for (int i = 0; i < objectDetectionIO.length; i++) {
                objectDetectionIO[i].updateInputs(objectDetectionInputs[i]);
                Logger.processInputs("Vision/ObjectDetectionCamera" + Integer.toString(i), objectDetectionInputs[i]);
            }
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over all apriltag cameras
        for (int cameraIndex = 0; cameraIndex < apriltagVisionIO.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!apriltagInputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : apriltagInputs[cameraIndex].tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : apriltagInputs[cameraIndex].poseObservations) {
                // Check whether to reject pose
                boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
                        || (observation.tagCount() == 1
                                && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                        || Math.abs(observation.pose().getZ()) > maxZError // Must have realistic Z coordinate

                        // Must be within the field boundaries
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > aprilTagLayout.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > aprilTagLayout.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                double angularStdDev = angularStdDevBaseline * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= linearStdDevMegatag2Factor;
                    angularStdDev *= angularStdDevMegatag2Factor;
                }
                if (cameraIndex < cameraStdDevFactors.length) {
                    linearStdDev *= cameraStdDevFactors[cameraIndex];
                    angularStdDev *= cameraStdDevFactors[cameraIndex];
                }

                // Send vision observation
                consumer.accept(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            // Log camera datadata
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
                    tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // loop over all object detection cameras
        updatingDetections = true;
        detections.clear();
        for (var detectionInput : objectDetectionInputs) {
            detections.addAll(List.of(detectionInput.detections));
        }
        updatingDetections = false;

        // Log summary data
        Logger.recordOutput(
                "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted",
                allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected",
                allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    }

    // returns all the objects the cameras can see
    // TODO: Add logic to compare results between both cameras and find duplicates
    public List<Detection> getAllObjectDetections() {
        while (updatingDetections)
            Thread.onSpinWait();
        return detections;
    }

    public void updateShooterRotation(double shooterAngle, double shooterRotationRadius) {
        for (ApriltagVisionIO io : apriltagVisionIO) {
            for (String affected : CAMERAS_AFFECTED_BY_SHOOTER_ROTATION) {
                if (Objects.equals(io.getPipelineName(), affected)) {
                    // TODO: some awesome calculations to update the bloody thing
                }
            }
        }
    }

    @FunctionalInterface
    public static interface ShooterAngleConsumer {
        public void accept(double shooterRotation, double shooterRotationRadius);
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
                Pose2d visionRobotPoseMeters,
                double timestampSeconds,
                Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
