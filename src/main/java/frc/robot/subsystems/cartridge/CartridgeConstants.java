package frc.robot.subsystems.cartridge;

public class CartridgeConstants {

    public static final int MOTOR_ID = 22;
    public static final int INNER_SWITCH_CHANNEL = 8;
    public static final int OUTER_SWITCH_CHANNEL = 0;
    public static final boolean INNER_NORMALLY_OPEN = false;
    public static final boolean OUTER_NORMALLY_OPEN = true;
    public static final int CURRENT_LIMIT = 90;
    public static final double VOLTAGE_COMPENSATION = 12;
    public static final double positionConversionFactor = 1.0 / 2.119;
    public static final double velocityConversionFactor = positionConversionFactor / 60.0;
    public static final boolean INVERTED = true;
    public static final double Kp = 0.6;
    public static final double Ki = 0;
    public static final double Kd = 0;
    public static final double Ks = 0.2;
    public static final double Kg = 1.2;
    public static final double Kv = 0.0;
    public static final double MAX_ACCELERATION = 10;
    public static final double MAX_VELOCITY = 4;
    public static final double TOLERANCE = 0.02;
    public static final double OPEN_CARTRIDGE_POS = 0.0;
    public static final double CLOSE_CARTRIDGE_POS = 1.0;
}
