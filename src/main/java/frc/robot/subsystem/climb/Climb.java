package frc.robot.subsystem.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    ClimbIOInputsAutoLogged climbIOInputs = new ClimbIOInputsAutoLogged();
    ClimbIO io;

    public Climb(ClimbIO io) {
        this.io = io;
    }

    public ClimbIO getIO() {
        return io;
    }

    @Override
    public void periodic() {
        io.updateInputs(climbIOInputs);
        Logger.processInputs("climb", climbIOInputs);
    }
}
