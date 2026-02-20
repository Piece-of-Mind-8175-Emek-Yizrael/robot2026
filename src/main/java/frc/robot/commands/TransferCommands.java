package frc.robot.Commands;

import static frc.robot.subsystems.transfer.TransferConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.transfer.Transfer;

public class TransferCommands {

    private Transfer transfer;

    public TransferCommands(Transfer transfer){
        this.transfer = transfer;
    }
    public Command setVoltage() {
        return Commands.runEnd(() -> transfer.getIO().setVoltage(TRANSFER_SPEED), () -> transfer.getIO().setVoltage(0), transfer);
    }

    public Command stopMotor() {
        return Commands.runOnce(() -> transfer.getIO().setVoltage(0));
    }

    public Command toggleTransfer(boolean on) {
        if (on) return Commands.runOnce(() -> transfer.getIO().setVelocity(TRANSFER_SPEED));
        return Commands.runOnce(() -> transfer.getIO().setVelocity(0));
    }
}
