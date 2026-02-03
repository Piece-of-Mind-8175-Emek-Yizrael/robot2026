package frc.robot.subsystems.cartridge;

import org.littletonrobotics.junction.AutoLog;

public interface CartridgeIO {

    @AutoLog

    public static class CartridgeIOInputs {
        public double voltage;
        public double velocity;
        public double output;
        public double postion;
        public boolean isInnerPressed;
        public boolean isOuterPressed;
    }

    public default void updateInputs(CartridgeIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void stop() {
    }

    public default boolean isInnerPressed() {
        return false;
    }

    public default boolean isOuterPressed() {
        return false;
    }

}
