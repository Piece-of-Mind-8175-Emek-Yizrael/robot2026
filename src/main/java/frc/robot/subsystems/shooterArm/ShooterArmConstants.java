package frc.robot.subsystems.shooterArm;

public class ShooterArmConstants {
    public static final int MOTOR_ID = 20;
    public static final int SENSOR_ID = 9;

    public static final int currentLimit = 40;
    public static final double voltageCompensation = 12.0;

    public static final double gearRatio = 100.0 / Math.PI * 2 * (3.0 / 7.0);
    public static final double velocityConversionFactor = gearRatio / 60.0; // RPM to rad/s

    public static final boolean INVERTED = false;

    public static final double TOLERANCE = 0.05; // radians
    public static final double MAX_VELOCITY = 10;//50
    public static final double MAX_ACCELERATION = 10;//50
    public static final double kp = 0.0;//2.0
    public static final double ki = 0.0;
    public static final double kd = 0.0;
    public static final double ks = 0.0;
    public static final double kv = 0.0;
    public static final double kg = 0.0;//0.7
    public static final double CLOSE_POS = 0.0;//FIXME: find actual values for these
    public static final double OPEN_POS = 1.0;

}
