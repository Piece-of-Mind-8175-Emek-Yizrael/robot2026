package frc.robot.subsystems.transfer;

import static frc.robot.subsystems.transfer.TransferConstants.*;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.POM_lib.Motors.POMSparkMax;

public class TransferIOReal implements TransferIO{
    POMSparkMax motor;
    RelativeEncoder encoder;

    public TransferIOReal() {
        motor = new POMSparkMax(TRANSFER_MOTOR_ID);

        encoder = motor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(SMART_CURRENT_LIMIT)
            .inverted(INVERTED);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(TransferIOInputs inputs) {
        inputs.velocity = encoder.getVelocity();
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
    public void setVelocity(double velocity) {
        motor.set(velocity);
    }
}
