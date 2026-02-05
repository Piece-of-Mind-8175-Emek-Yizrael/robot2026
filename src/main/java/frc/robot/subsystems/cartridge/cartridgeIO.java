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
        public boolean atGoal;
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

    public default void goToPos(double goal) {
    }

    public default boolean atGoal() {
        return false;
    }

    public default double getPos() {
        return 0;
    }

    public default void resetPID() {
    }

    public default void setPIDValues() {
    }

}
