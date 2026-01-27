package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs{
        public double intakeVoltage = 0.0;
    }
    public default void updateInputs(IntakeIOInputs inputs){}  

    public default void setVoltage(double Voltage){} 

    public default void stopMotor(){}
}
