package frc.robot.subsystems.intake;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.POM_lib.Motors.POMSparkMax;

public class IntakeIOSparkMax implements IntakeIO{
    private final POMSparkMax motor;
    private final SparkMaxConfig config;
    

    public IntakeIOSparkMax(){
        motor = new POMSparkMax(0);
        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        inputs.intakeVoltage = motor.getAppliedOutput();
    }

    @Override
    public void setVoltage(double Voltage) {
        motor.setVoltage(Voltage);
    }

    @Override
    public void StopMotor() {
        motor.stop();
    }
}
