package frc.robot.subsystems.cartridge;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;
import static frc.robot.subsystems.cartridge.cartridgeConstants.*;

public class cartridgeIOSparkMax implements cartridgeIO {
    private final POMSparkMax motor;
    private final POMDigitalInput innerSwitch;
    private final POMDigitalInput outerSwitch;
    private final SparkMaxConfig config;

    public cartridgeIOSparkMax() {
        motor = new POMSparkMax(MOTOR_ID);
        innerSwitch = new POMDigitalInput(INNER_SWITCH_CHANNEL, INNER_NORMALLY_OPEN);
        outerSwitch = new POMDigitalInput(OUTTER_SWITCH_CHANNEL, OUTTER_NORMALLY_OPEN);
        config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake)
                .smartCurrentLimit(STALL_LIMIT)
                .voltageCompensation(12);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(cartridgeIOInputs inputs) {
        inputs.voltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.output = motor.getAppliedOutput();
        inputs.isInnerPressed = isInnerPressed();
        inputs.isOutterPressed = isOutterPressed();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        motor.stop();
    }

    @Override
    public boolean isInnerPressed() {
        return innerSwitch.get();
    }

    @Override
    public boolean isOutterPressed() {
        return outerSwitch.get();
    }

}
