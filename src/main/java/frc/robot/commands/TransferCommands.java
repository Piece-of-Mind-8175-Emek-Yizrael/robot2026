package frc.robot.commands;

import static frc.robot.subsystems.transfer.TransferConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.transfer.Transfer;

public class TransferCommands {

    private Transfer transfer;

    public TransferCommands(Transfer transfer){
        this.transfer = transfer;
    }
    public Command setVoltage(double voltage) {
        return Commands.runEnd(() -> transfer.getIO().setVoltage(voltage), () -> transfer.getIO().setVoltage(0), transfer);
    }

    public Command setForwoard() {
        double voltage = 5.0;
        return setVoltage(voltage);
    }
    
    public Command setBackward() {
        double voltage = -5.0;
        return setVoltage(voltage);
    }

    public Command stopMotor() {
        return Commands.runOnce(() -> transfer.getIO().setVoltage(0));
    }

    public Command toggleTransfer(boolean on) {
        if (on) return Commands.runOnce(() -> transfer.getIO().setVelocity(TRANSFER_SPEED));
        return Commands.runOnce(() -> transfer.getIO().setVelocity(0));
    }
}
