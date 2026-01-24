package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs{
        public double intakeVoltage = 0.0;
        public double intakeVelocity = 0.0;
        public boolean intakeSensor = false;
    }
    public default void updateInputs(IntakeIOInputs inputs){}  

    public default void setVoltage(double Voltage){} 

    public default void SetVelocity(double Velocity){}

    public default void stopMotor(){}
}
