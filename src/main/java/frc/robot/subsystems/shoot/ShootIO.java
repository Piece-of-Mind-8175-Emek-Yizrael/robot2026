package frc.robot.subsystems.shoot;

import org.littletonrobotics.junction.AutoLog;

public interface ShootIO {

    @AutoLog
    public static class ShootIOInputs {

        public double goalHoodVelocity = 0.0;
        //left hood motor
        public boolean leftHoodConnected = false;
        public double leftVoltage = 0.0;
        public double leftVelocity = 0.0;
        public double leftAppliedVoltage = 0.0;

        //right hood motor
        public boolean rightHoodConnected = false;
        public double rightVoltage = 0.0;
        public double rightVelocity = 0.0;
        public double rightAppliedVoltage = 0.0;

        //transfer motor
        public boolean feedConnected = false;
        public double feedVoltage = 0.0;
        public double feedVelocity = 0.0;
        public double feedAppliedVoltage = 0.0;

    }

    public default void updateInputs(ShootIOInputs inputs) {
    }

    public default void setHoodVoltage(double voltage) {
    }

    public default void setFeedVoltage(double voltage) {
    }

    public default void stopHood() {
    }

    public default void stopFeed() {
    }

    public default void setBoth(double hoodVoltage, double transferVoltage) {
    }

    public default void stopBoth() {
    }

    public default void setHoodSetpoint(double goal){
    }

    public default boolean atGoal(){
        return false;
    }
    
}
