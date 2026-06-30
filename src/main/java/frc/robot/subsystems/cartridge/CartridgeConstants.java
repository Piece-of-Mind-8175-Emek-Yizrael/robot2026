package frc.robot.subsystems.cartridge;

public class CartridgeConstants {

    public static final int MOTOR_ID = 22;
    public static final int INNER_SWITCH_CHANNEL = 8;
    public static final int OUTER_SWITCH_CHANNEL = 0;
    public static final boolean INNER_NORMALLY_OPEN = false;
    public static final boolean OUTER_NORMALLY_OPEN = true;
    public static final int CURRENT_LIMIT = 90;
    public static final double VOLTAGE_COMPENSATION = 12;
    public static final double CONVERSION_FACTOR = (3 / 7);
    public static final double VELOCITY_CONVERSION_FACTOR = CONVERSION_FACTOR / 60.0;
    public static final boolean INVERTED = true;

    public static final double Kp = 0.0;
    public static final double Ki = 0.0;
    public static final double Kd = 0;
    public static final double Ks = 0.0;
    public static final double Kg = 0.0;
    public static final double Kv = 0.0;
    public static final double MAX_ACCELERATION = 10;
    public static final double MAX_VELOCITY = 4;
    public static final double TOLERANCE = 0.02;

    // public static final double closeCartridgePos = 1.0;
    public static final double closeCartridgePos = 1.0;
    public static final double openCartridgePos = 0.0;

    public static final double closeCartridgeVolt = -2.5;
    public static final double openCartridgeVolt = 1.5;

    public static final double shakeCloseCartridgeVolt = -3.0;
    public static final double shakeOpenCartridgeVolt = 2.0;
    public static final double shakeTimeOut = 0.4;

}
