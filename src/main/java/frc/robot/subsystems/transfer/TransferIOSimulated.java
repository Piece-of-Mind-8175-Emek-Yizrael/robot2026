package frc.robot.subsystems.transfer;

public class TransferIOSimulated implements TransferIO {
    // TODO: implement the simulated transferIO

    @Override
    public void updateInputs(TransferIOInputs inputs) {

    }

    @Override
    public void stopMotor() {
        setVoltage(0);
    }

    @Override
    public void setVoltage(double voltage) {

    }

    @Override
    public void setPercentOfSpeed(double percentage) {

    }
}
