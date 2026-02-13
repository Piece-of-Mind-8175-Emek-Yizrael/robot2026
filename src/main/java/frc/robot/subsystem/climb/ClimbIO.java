package frc.robot.subsystem.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

    @AutoLog

    public static class ClimbIOInputs {
        public static double voltage;
        public static double output;
        public static double angle;
        public static boolean isPressed;
    }

    public default void updateInputs(ClimbIOInputs inputs) {
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

    public default void preClimbGoToPos(double goal) {
    }

    public default void ClimbGoToPos(double goal) {
    }

    public default boolean atGoal() {
        return false;
    }

    public default double getPosition() {
        return 0;
    }

}
