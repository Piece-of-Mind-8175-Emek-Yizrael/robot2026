package frc.robot.subsystems.transfer;

import org.littletonrobotics.junction.AutoLog;

public interface TransferIO {
    @AutoLog
    public static class TransferIOInputs {
        public double velocity;
        public double voltage;
    }

    public void stopMotor();
    public void updateInputs(TransferIOInputs inputs);
    public void setVoltage(double voltage);
    public void setPercentOfSpeed(double percentage);
}
