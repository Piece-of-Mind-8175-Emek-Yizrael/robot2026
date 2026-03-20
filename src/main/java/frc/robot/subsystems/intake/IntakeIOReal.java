package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.CURRENT_LIMIT;
import static frc.robot.subsystems.intake.IntakeConstants.MOTOR_ID;
import static frc.robot.subsystems.intake.IntakeConstants.RAMP_RATE;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.POM_lib.Motors.POMTalonFX;

public class IntakeIOReal implements IntakeIO{
    private final POMTalonFX motor;
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private VoltageOut voltageOut = new VoltageOut(0);

    public IntakeIOReal(){
        motor = new POMTalonFX(MOTOR_ID);
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod =  RAMP_RATE;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor.getConfigurator().apply(config);        
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.intakeVelocity = motor.getVelocity().getValueAsDouble();
        inputs.intakeAmp = motor.getSupplyCurrent().getValueAsDouble();
        
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
