package frc.robot.subsystems.transfer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Transfer extends SubsystemBase {
    TransferIO io;
    TransferIOInputsAutoLogged inputs = new TransferIOInputsAutoLogged();

    public Transfer(TransferIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Transfer", inputs);
    }

    public TransferIO getIO() {
        return io;
    }

}
