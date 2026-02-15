package frc.robot.commands;

import static frc.robot.subsystems.transfer.TransferConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.transfer.Transfer;

public class TransferCommands {
    public Command setVoltage(Transfer subsystem, double voltage) {
        return Commands.runEnd(() -> subsystem.getIO().setVoltage(voltage), () -> subsystem.getIO().setVoltage(0), subsystem);
    }

    public Command stopMotor(Transfer subsystem) {
        return Commands.runOnce(() -> subsystem.getIO().setVoltage(0));
    }

    public Command toggleTransfer(Transfer subsystem, boolean on) {
        if (on) return Commands.runOnce(() -> subsystem.getIO().setVelocity(TRANSFER_SPEED));
        return Commands.runOnce(() -> subsystem.getIO().setVelocity(0));
    }
}
