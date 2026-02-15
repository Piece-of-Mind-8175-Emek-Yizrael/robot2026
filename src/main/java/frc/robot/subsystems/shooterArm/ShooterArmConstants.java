package frc.robot.subsystems.shooterArm;

public class ShooterArmConstants {
    public static final int MOTOR_ID = 20;
    
    public static final int currentLimit = 40;
    public static final double voltageCompensation = 12.0;

    public static final double gearRatio = ((1.0 / 10.0) * (18.0 / 42.0)) ;
    public static final double velocityConversionFactor = gearRatio / 60.0; // RPM to rad/s

    public static final boolean INVERTED = false;
    
    public static final double TOLERANCE = 0.01;
    public static final double MAX_VELOCITY = 1;
    public static final double MAX_ACCELERATION = 1;
    public static final double kp = 0.0;
    public static final double ki = 0.0;
    public static final double kd = 0.0;
    public static final double ks = 0.0;
    public static final double kv = 0.0;
    public static final double kg = 0.0;
    
    public static final double CLOSE_POS = 5.0;
    public static final double OPEN_POS = 50.0;
    
}
