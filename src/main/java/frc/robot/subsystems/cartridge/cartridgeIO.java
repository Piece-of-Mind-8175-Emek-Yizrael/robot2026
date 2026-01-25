package frc.robot.subsystems.cartridge;

import org.littletonrobotics.junction.AutoLog;

public interface cartridgeIO {

    @AutoLog

    public static class cartridgeIOInputs {
        public double voltage;
        public double velocity;
        public double output;
        public double postion;
        public boolean isPressed;
    }

    public default void updateInputs(cartridgeIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void stop() {
    }

    public default void goToPosition(double goal) {
    }

    public default boolean isPressed() {
        return false;
    }

    public default void setPIDValues() {
    }

}
