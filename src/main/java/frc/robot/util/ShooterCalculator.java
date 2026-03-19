package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterCalculator {
    private static InterpolatingDoubleTreeMap farSpeeds, closeSpeeds;
    private static InterpolatingDoubleTreeMap flyTime;

    private static final double FAR_MIN_DISTANCE = 2.0;

    public static void init() {
        closeSpeeds = new InterpolatingDoubleTreeMap();
        closeSpeeds.put(1.7, 42.0);
        closeSpeeds.put(2.0, 43.0);

        farSpeeds = new InterpolatingDoubleTreeMap();
        farSpeeds.put(2.0, 43.0);
        farSpeeds.put(2.1, 44.0);
        farSpeeds.put(2.2, 44.0);
        farSpeeds.put(2.4, 46.0);
        farSpeeds.put(2.5, 48.0);
        farSpeeds.put(2.6, 49.0);
        farSpeeds.put(2.8, 51.0);
        farSpeeds.put(3.0, 52.0);
        farSpeeds.put(3.2, 53.0);
        farSpeeds.put(3.5, 56.0);
        // farSpeeds.put(3.7, 58.0);//TODO need to change
        // farSpeeds.put(4.0, 59.0);//TODO need to change
        // farSpeeds.put(4.2, 61.0);//TODO need to change
        // farSpeeds.put(4.5, 61.0);//TODO need to change
        // farSpeeds.put(4.7, 62.0);
        // farSpeeds.put(5.0, 63.0);
        // farSpeeds.put(5.2, 65.0);

        flyTime = new InterpolatingDoubleTreeMap();
        flyTime.put(43.0,0.72);
        flyTime.put(44.0,0.8);
        flyTime.put(46.0,0.87);
        flyTime.put(48.0,1.1);
        flyTime.put(49.0,1.2);
        flyTime.put(51.0,1.21);
        flyTime.put(52.0,1.21);
        flyTime.put(53.0,1.17);
        flyTime.put(56.0,1.32);
        // flyTime.put(58.0,0.0);//TODO need to change
        // flyTime.put(59.0,0.0);//TODO need to change
        // flyTime.put(61.0,0.0);//TODO need to change
        // flyTime.put(62.0,0.0);
        // flyTime.put(63.0,0.0);
        // flyTime.put(65.0,0.0);
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

    public static InterpolatorResult getTargetSpeedAndRotation(double radialDistance, double radialVelocity, double perpendicularVelocity) {
        double radialError = flyTime.get(farSpeeds.get(radialDistance)) * radialVelocity;
        radialDistance += radialError;

        double flightTime = flyTime.get(farSpeeds.get(radialDistance));

        double perpendicularError = perpendicularVelocity * flightTime;

        double normalizedDistance = Math.sqrt(perpendicularError * perpendicularError + radialDistance * radialDistance);

        double angle = Math.atan2(perpendicularError, radialDistance);
        double velocity = farSpeeds.get(normalizedDistance);

        return new InterpolatorResult(velocity, angle);
    }
}
