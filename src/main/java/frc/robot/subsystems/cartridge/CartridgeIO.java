package frc.robot.subsystems.cartridge;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.CartridgePose;

public interface CartridgeIO {

    @AutoLog
    public static class CartridgeIOInputs {
        public double voltage = 0;
        public double velocity = 0;
        public double output = 0;
        public boolean isClosePressed = false;
        public boolean isOpenPressed = false;
    }

    public default void updateInputs(CartridgeIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void stop() {
    }

    public default boolean isClosePressed() {
        return false;
    }

    public default boolean isOpenPressed() {
        return false;
    }

    public default CartridgePose getCartridgePose(){
        return CartridgePose.IN_MOVEMENT;
    }
}
