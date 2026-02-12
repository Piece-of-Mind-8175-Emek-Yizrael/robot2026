package frc.robot.subsystem.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

    @AutoLog

    public static class ClimbIOInputs {
        public double voltage;
        public double output;
        public double angle;
        public boolean isPressed;
    }

    public default void updateInputs(ClimbIOInputs inputs) {
    }

    public default void setClimbVoltage(double voltage) {
    }
    
    public default void stopClimb() {
    }
    
    public default void setArmMotor(double voltage) {
    }
    
    public default void stopArm() {
    }

    public default void setServoAngle(double angle) {
    }

    public default boolean isPressed() {
        return false;
    }

}
