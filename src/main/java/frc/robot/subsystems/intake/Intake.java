package frc.robot.subsystems.intake;

import java.lang.System.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged ();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public IntakeIO getIO(){
        return io;
    }
    
    @Override
    public void periodic(){
        io.updateInpunts(inputs);
        Logger.processInputs("Intake", inputs);
    }
}

