package frc.robot.subsystems.shooterArm;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterArmIO {

    @AutoLog
    public static class ShooterArmIOInputs {
        public boolean motorConnected = false;
        public double armPosition = 0.0;
        public double armVelocity = 0.0;
        public double motorVoltage = 0.0;
        public double motorAppliedVoltage = 0.0;
    }

    public default void updateInputs(ShooterArmIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void stopMotor() {
    }

    public default void setGoal(double goal) {
    }

    public default boolean atGoal() {
        return true;
    }

    public default void zeroPosition() {
    }

    public default void resistGravity() {
    }

    public default double getPos(){
        return 0.0;
    }
    
    public default void resetPID() {
    }

    public default void resetPID(double newGoal) {
    }

    public default void stayInCurrentGoal() {
    }
}
