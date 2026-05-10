package frc.robot.subsystems.shoot;

import com.ctre.phoenix6.signals.InvertedValue;

public class ShootConstants {

    public static final int LEFT_HOOD_MOTOR_ID = 15;
    public static final int RIGHT_HOOD_MOTOR_ID = 16;
    public static final int FEED_MOTOR_ID = 50;
    public static final double hoodGearRatio = 1.0;
    public static final InvertedValue RIGHT_HOOD_DIRECTION = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue LEFT_HOOD_DIRECTION = InvertedValue.Clockwise_Positive;
    public static final double slipCurrent = 40.0; // Amps
    public static final double rampRate = 0.25; // seconds from 0 to full
    public static final int feedCurrentLimit = 90; // Amps
    public static final double feedGearRatio = 1.0 / 9.0; // rotations
    public static final double feedEncoderVelocityFactor = feedGearRatio / 60.0; // rotatio per second
    public static final boolean feedInverted = false; // true is inverted

    public static final double FEED_SHOOT_VOLTAGE = 4.0;

    public static final double kvRightHood = 0.115;
    public static final double ksRightHood = 0.2;
    public static final double kpRightHood = 1.2;
    public static final double kiRightHood = 0.0;
    public static final double kdRightHood = 0.0;

    public static final double kvLeftHood = 0.115;
    public static final double ksLeftHood = 0.2;
    public static final double kpLeftHood = 0.5;
    public static final double kiLeftHood = 0.0;
    public static final double kdLefthHood = 0.0;

    public static final double kpFeed = 1.2;
    public static final double kiFeed = 0.0;
    public static final double kdFeed = 0.0;
    public static final double maxAccelerationFeed = 35.0;
    public static final double maxVelocityFeed = 35.0;
    public static final double feedTolerance = 0.2; // rotations per second

    public static final double hoodTolerance = 2; // rotations per second

    public static final double FEED_SHOOT_SETPOINT = 1.0; // rotations per second

    public static final double feedVol = 6.0;
    public static final double deliveryVelocity = 55.0;
}
