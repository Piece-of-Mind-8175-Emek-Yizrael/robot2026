package frc.robot.subsystems.shoot;

import com.ctre.phoenix6.signals.InvertedValue;

public class ShootConstants {

    public static final int LEFT_HOOD_MOTOR_ID = 15;
    public static final int RIGHT_HOOD_MOTOR_ID = 16;
    public static final int FEED_MOTOR_ID = 20;
    public static final double hoodGearRatio = 1.0 / 9.0; 
    public static final InvertedValue HOOD_DIRECTION = InvertedValue.CounterClockwise_Positive;
    public static final double slipCurrent = 30.0; //Amps
    public static final double rampRate = 0.25; //seconds from 0 to full
    public static final int feedCurrentLimit = 30; //Amps
    public static final double feedGearRatio = 1.0; //rotations
    public static final double feedEncoderVelocityFactor = feedGearRatio / 60.0; //rotatio per second
    public static final boolean feedInverted = false; //true is inverted

    public static final double FEED_SHOOT_VOLTAGE = 4.0;

    
    public static final double kvHood = 0.0;
    public static final double ksHood = 0.0;
    public static final double kpHood = 0.0;
    public static final double kiHood = 0.0;
    public static final double kdHood = 0.0;
    
    
    public static final double kpFeed = 0.0;
    public static final double kiFeed = 0.0;
    public static final double kdFeed = 0.0;
    public static final double maxAccelerationFeed = 1.0;
    public static final double maxVelocityFeed = 1.0;
    public static final double feedTolerance = 0.2; //rotations per second
    
    public static final double hoodTolerance = 0.2; //rotations per second

    public static final double FEED_SHOOT_SETPOINT = 1.0; //rotations per second
}
