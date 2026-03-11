package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterCalculator {
    private static InterpolatingDoubleTreeMap farSpeeds, closeSpeeds;

    private static final double FAR_MIN_DISTANCE = 2.5;

    public static void init() {
        closeSpeeds = new InterpolatingDoubleTreeMap();
        closeSpeeds.put(1.0, 50.0);
        closeSpeeds.put(2.0, 55.0);

        farSpeeds = new InterpolatingDoubleTreeMap();
        farSpeeds.put(3.0, 50.0);
        farSpeeds.put(3.5, 55.0);
    }

    public static boolean isFar(double distance) {
        return distance > FAR_MIN_DISTANCE;
    }

    public static double getTargetSpeed(double distance) {
        if (isFar(distance)) {
            return farSpeeds.get(distance);
        }

        return closeSpeeds.get(distance);
    }

}
