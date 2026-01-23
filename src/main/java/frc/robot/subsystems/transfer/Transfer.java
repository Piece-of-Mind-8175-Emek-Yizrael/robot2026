package frc.robot.subsystems.transfer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transfer extends SubsystemBase {
    TransferIO io;

    public Transfer(TransferIO io) {
        this.io = io;
    }

    public TransferIO getIO() {
        return io;
    }
}
