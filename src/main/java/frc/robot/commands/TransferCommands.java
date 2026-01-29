package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferConstants;

public class TransferCommands {
    public static Command setVoltage(Transfer subsystem, double voltage) {
        return Commands.runOnce(() -> subsystem.getIO().setVoltage(voltage));
    }

    public static Command stopMotor(Transfer subsystem) {
        return Commands.runOnce(() -> subsystem.getIO().setVoltage(0));
    }

    public static Command toggleTransfer(Transfer subsystem, boolean on) {
        if (on) return Commands.runOnce(() -> subsystem.getIO().setPercentOfSpeed(TransferConstants.TRANSFER_SPEED));
        return Commands.runOnce(() -> subsystem.getIO().setPercentOfSpeed(0));
    }
}
