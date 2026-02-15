package frc.robot.subsystem.climb2;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;

public interface ClimbIO {

    @AutoLog
    public static class ClimbIOInputs {
        public Voltage krakenVoltage;
        public double KrakenOutput;
        public double krakenPosition;
        public boolean krakenAtGoal;

        public double neoVoltage;
        public double neoOutput;
        public double neoAngle;

        public double servoAngle;

        public boolean isPressed;
    }

    public default void updateInputs(ClimbIOInputs inputs) {
    }

    public default void setKrakenVoltage(double voltage) {
    }

    public default void stopKraken() {
    }

    public default void setNeoVoltage(double voltage) {
    }

    public default void stopNeo() {
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

    public default boolean atKrakenGoal() {
        return false;
    }

    public default double getPosition() {
        return 0;
    }

    public default void resetPos(){
        
    }

    public default void setNeoGoal(double goal) {

    }

    public default boolean atNeoGoal() {
        return false;
    }
    
}
