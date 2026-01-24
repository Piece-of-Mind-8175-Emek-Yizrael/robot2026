package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.POM_lib.Motors.POMTalonFX;

import static frc.robot.subsystems.intake.IntakeConstants.MOTOR_ID;

public class IntakeIOKraken implements IntakeIO{
    private final POMTalonFX intakeKraken;
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    public IntakeIOKraken(){
        intakeKraken = new POMTalonFX(MOTOR_ID);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        IntakeIO.super.updateInputs(inputs);
    }

    @Override
    public void setVoltage(double Voltage) {
        IntakeIO.super.setVoltage(Voltage);
    }

    @Override
    public void SetVelocity(double Velocity) {
        IntakeIO.super.SetVelocity(Velocity);
    }
    
    @Override
    public void stopMotor() {
        IntakeIO.super.stopMotor();
    }
}
