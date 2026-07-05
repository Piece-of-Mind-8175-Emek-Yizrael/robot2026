package frc.robot.subsystems.cartridge;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.POM_lib.Motors.POMTalonFX;
import frc.robot.POM_lib.sensors.POMDigitalInput;

import static frc.robot.subsystems.cartridge.CartridgeConstants.*;

public class CartridgeIOReal implements CartridgeIO{
    private final POMTalonFX motor;
    private final POMDigitalInput closeSwitch;
    private final POMDigitalInput openSwitch;
    private final TalonFXConfiguration config;

    public CartridgeIOReal(){
        motor = new POMTalonFX(MOTOR_ID);
        closeSwitch = new POMDigitalInput(CLOSE_SWITCH_CHANNEL);
        openSwitch = new POMDigitalInput(OPEN_SWITCH_CHANNEL);

        config = new TalonFXConfiguration();

        config.Feedback.SensorToMechanismRatio = CONVERSION_FACTOR;
        config.TorqueCurrent.PeakForwardTorqueCurrent = CURRENT_LIMIT;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        motor.getConfigurator().apply(config);

    }

    @Override
    public void updateInputs(CartridgeIOInputs inputs) {
        inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.output = motor.getDutyCycle().getValueAsDouble();
        inputs.velocity = motor.getVelocity().getValueAsDouble();
        inputs.isClosePressed = isClosePressed();
        inputs.isOpenPressed = isOpenPressed();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void stop() {
        motor.stop();
    }

    @Override
    public boolean isClosePressed() {
        return closeSwitch.get();
    }

    @Override
    public boolean isOpenPressed() {
        return openSwitch.get();
    }
}
