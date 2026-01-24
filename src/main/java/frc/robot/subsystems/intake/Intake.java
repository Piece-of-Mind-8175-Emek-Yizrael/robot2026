package frc.robot.subsystems.intake;

import java.lang.System.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged ();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInpunts(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public void setVoltage(double voltage){
        io.setVoltage(voltage);
    }

    public void SetVelocity(double Velocity){
        io.SetVelocity(Velocity);
    }
    
    public void stopMotor(){
        io.stopMotor();
    }
}
