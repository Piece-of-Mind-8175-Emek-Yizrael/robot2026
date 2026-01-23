package frc.robot.subsystems.transfer;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import frc.robot.POM_lib.Motors.POMSparkMax;

public class TransferIOReal implements TransferIO{
    SparkMax motor;

    public TransferIOReal() {
        motor = new SparkMax(TransferConstants.TRANSFER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        // some motor = new some motor type;
    }

    @Override
    public void updateInputs(TransferIOInputs inputs) {

    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setPercentOfSpeed(double percentage) {
        motor.set(percentage);
    }
}
