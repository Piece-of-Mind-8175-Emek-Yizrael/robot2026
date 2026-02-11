package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;

import frc.robot.POM_lib.Motors.POMSparkMax;

public class IntakeIOSparkMax implements IntakeIO{
    private final POMSparkMax motor;
    private RelativeEncoder encoder;

    public IntakeIOSparkMax(){
        motor = new POMSparkMax(0);
        encoder = motor.getEncoder();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        inputs.intakeVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.intakeVelocity = encoder.getVelocity();
        inputs.intakeAmp = motor.getAppliedOutput();
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
