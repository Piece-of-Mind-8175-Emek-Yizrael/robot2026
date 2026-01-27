package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.MOTOR_ID;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.POM_lib.Motors.POMTalonFX;

public class IntakeIOKraken implements IntakeIO{
    private final POMTalonFX motor;
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private VoltageOut voltageOut = new VoltageOut(0);

    public IntakeIOKraken(){
        motor = new POMTalonFX(MOTOR_ID);
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(config);
        
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeVoltage = motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void stopMotor() {
        setVoltage(0);
    }
}
