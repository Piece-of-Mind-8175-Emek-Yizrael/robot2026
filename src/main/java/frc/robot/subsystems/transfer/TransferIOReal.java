package frc.robot.subsystems.transfer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.POM_lib.Motors.POMSparkMax;

public class TransferIOReal implements TransferIO{
    SparkMax motor;

    public TransferIOReal() {
        motor = new SparkMax(TransferConstants.TRANSFER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(50);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(TransferIOInputs inputs) {
        inputs.velocity = motor.getEncoder().getVelocity();
        inputs.voltage = (motor.getAppliedOutput() * motor.getBusVoltage());
    }

    @Override
    public void stopMotor() {
        setVoltage(0);
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
