package frc.robot.subsystems.shoot;

import org.littletonrobotics.junction.AutoLog;

public interface ShootIO {

    @AutoLog
    public static class ShootIOInputs {
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
        public boolean transferConnected = false;
        public double transferVoltage = 0.0;
        public double transferVelocity = 0.0;
        public double transferAppliedVoltage = 0.0;

    }

    public default void updateInputs(ShootIOInputs inputs) {
    }

    public default void setHoodVoltage(double voltage) {
    }

    public default void setTransferVoltage(double voltage) {
    }

    public default void stopHood() {
    }

    public default void stopTransfer() {
    }

    public default void setVoltage(double hoodVoltage, double transferVoltage) {
    }

    public default void stopBoth() {
    }

    public default void setGoal(double goal){
    }

    public default boolean atGoal(){
        return false;
    }

    

    
}
