package frc.robot.subsystems.shoot;

public class ShootConstants {

    public static final int LEFT_HOOD_MOTOR_ID = 0; //TODO: change motor ID
    public static final int RIGHT_HOOD_MOTOR_ID = 0; //TODO: change motor ID
    public static final int TRANSFER_MOTOR_ID = 0; //TODO: change motor ID
    public static final double encoderPositionFactor = 1.0; 
    public static final double slipCurrent = 30.0; //Amps
    public static final double rampRate = 0.5; //seconds from 0 to full
    public static final int transferCurrentLimit = 10; //Amps
    public static final double transferEncoderPositionFactor = 1.0; //rotations
    public static final double transferEncoderVelocityFactor = transferEncoderPositionFactor / 60.0; //rotations per second
    
    public static final double kv = 1.0;
    public static final double ks = 1.0;
    public static final double kp = 1.0;
    public static final double ki = 1.0;
    public static final double kd = 1.0;
}
