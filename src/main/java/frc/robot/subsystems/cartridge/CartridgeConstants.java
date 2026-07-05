package frc.robot.subsystems.cartridge;

public class CartridgeConstants {
    public static final int MOTOR_ID = 22;
    public static final int CLOSE_SWITCH_CHANNEL = 1;
    public static final int OPEN_SWITCH_CHANNEL = 0;
    public static final int CURRENT_LIMIT = 90;
    public static final double VOLTAGE_COMPENSATION = 12;
    public static final double CONVERSION_FACTOR = (3 / 7);
    public static final double VELOCITY_CONVERSION_FACTOR = CONVERSION_FACTOR / 60.0;
    public static final boolean INVERTED = true;


    public static final double closeCartridgePos = 1.0;
    public static final double openCartridgePos = 0.0;

    public static final double closeCartridgeVolt = -1.5;
    public static final double openCartridgeVolt = 1.5;

    public static final double shakeCloseCartridgeVolt = -1.5;
    public static final double shakeOpenCartridgeVolt = 1.5;
    public static final double shakeTimeOut = 0.4;
}
