package frc.robot.subsystems.shoot;

public class ShootConstants {

    public static final int LEFT_HOOD_MOTOR_ID = 0; //TODO: change motor ID
    public static final int RIGHT_HOOD_MOTOR_ID = 0; //TODO: change motor ID
    public static final int FEED_MOTOR_ID = 0; //TODO: change motor ID
    public static final double hoodGearRatio = 1.0; 
    public static final double slipCurrent = 30.0; //Amps
    public static final double rampRate = 0.1; //seconds from 0 to full
    public static final int feedCurrentLimit = 10; //Amps
    public static final double feedGearRatio = 1.0; //rotations
    public static final double feedEncoderVelocityFactor = feedGearRatio / 60.0; //rotations per second
    
    
    public static final double kv = 1.0;
    public static final double ks = 1.0;
    public static final double kp = 1.0;
    public static final double ki = 1.0;
    public static final double kd = 1.0;

    public static final double hoodTolerance = 0.2; //rotations per second
}
