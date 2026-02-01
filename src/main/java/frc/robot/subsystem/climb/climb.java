package frc.robot.subsystem.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climb extends SubsystemBase {
    climbIOInputsAutoLogged climbIOInputs = new climbIOInputsAutoLogged();
    climbIO io;

    public climb(climbIO io) {
        this.io = io;
    }

    public climbIO getIO() {
        return io;
    }

    @Override
    public void periodic() {
        io.updateInputs(climbIOInputs);
        Logger.processInputs("climb", climbIOInputs);
    }
}
