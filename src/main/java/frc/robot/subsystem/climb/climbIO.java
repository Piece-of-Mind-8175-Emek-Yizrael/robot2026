package frc.robot.subsystem.climb;

import org.littletonrobotics.junction.AutoLog;

public interface climbIO {

    @AutoLog

    public static class climbIOInputs {
        public static double voltage;
        public static double output;
        public static double angle;
        public static boolean isPressed;
    }

    public default void updateInputs(climbIOInputs inputs) {
    }

    public default void setMotorVoltage(double voltage) {
    }

    public default void stopMotor() {
    }

    public default void setServoAngle(double angle) {
    }

    public default boolean isPressed() {
        return false;
    }

}
