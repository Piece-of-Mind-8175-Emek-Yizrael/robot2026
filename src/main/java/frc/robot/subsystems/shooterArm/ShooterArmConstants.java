package frc.robot.subsystems.shooterArm;

import edu.wpi.first.math.util.Units;

public class ShooterArmConstants {
    public static final int MOTOR_ID = 20;
    public static final int SENSOR_ID = 9;

    public static final int currentLimit = 40;
    public static final double voltageCompensation = 12.0;

    public static final double gearRatio = (2 * Math.PI) / (3.0 / 7.0);
    public static final double velocityConversionFactor = gearRatio / 60.0; // RPM to rad/s

    public static final boolean INVERTED = false;

    public static final double TOLERANCE = 0.05; // radians
    public static final double MAX_VELOCITY = 10;//50
    public static final double MAX_ACCELERATION = 10;//50
    public static final double kp = 0.9;//2.0
    public static final double ki = 0.0;
    public static final double kd = 0.0;
    public static final double ks = 0.0;
    public static final double kv = 0.0;
    public static final double kg = 0.7;//0.7
    public static final double CLOSE_POS = Units.degreesToRadians(-54);//FIXME: find actual values for these
    public static final double OPEN_POS = 2.548;

}
