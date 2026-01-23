package frc.robot.subsystems.transfer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transfer extends SubsystemBase {
    TransferIO io;

    public Transfer(TransferIO io) {
        this.io = io;
    }

    public TransferIO getIO() {
        return io;
    }

    public Command toggleTransfer(boolean on) {
        if (on) return runOnce(() -> io.setPercentOfSpeed(TransferConstants.TRANSFER_SPEED));
        return runOnce(() -> io.setPercentOfSpeed(0));
    }
}
