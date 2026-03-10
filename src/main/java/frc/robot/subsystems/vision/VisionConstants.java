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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

// TODO: replace last years variables with this year's variables (2025 to 2026)
public class VisionConstants {

    public static enum TargetType {
        FUEL(0);

        final int classId;

        private TargetType(int classId) {
            this.classId = classId;
        }

        public int getClassId() {
            return classId;
        }
    }
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera names, must match names configured on coprocessor
    public static String backCameraName = "first_camera";
    public static String frontCameraName = "seconde_camera";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)2921
    public static Transform3d InitialRobotToShooterCameraTranslation = new Transform3d(0.2921, 0.0,0.515, new Rotation3d(0, Units.degreesToRadians(-35.0),0));    
    public static Transform3d CAMERA_TO_ROBOT_CLOSED_CARTRIDGE_TRANSLATION = new Transform3d(0.338, 0.0,0, new Rotation3d(0.0,0.0, 0.0)); // TODO: replace this with the actual value
    public static Transform3d CAMERA_TO_ROBOT_OPEN_CARTRIDGE_TRANSLATION = new Transform3d(0.613,0.0, 0.466, new Rotation3d(0.0,0.0, 0.0)); // TODO: replace this with the actual value
    
    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.5; // Meters
    public static double angularStdDevBaseline = .7; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
            1.0, // Camera 0
            1.0 // Camera 1
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available

    public static class ObjectDetectionConstants {
        public static double FUEL_DIAMETER_METERS = 0.150114; // TODO: replace this with the diameter of the 2026 fuel object

    }
}