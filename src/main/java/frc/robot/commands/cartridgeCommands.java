package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.cartridge.Cartridge;
import static frc.robot.subsystems.cartridge.CartridgeConstants.*;

public class CartridgeCommands extends Command {

    Cartridge cartridge;

    public CartridgeCommands(Cartridge cartridge) {
        this.cartridge = cartridge;
    }

    public Command setVoltage(double voltage) {
        return Commands.runEnd(() -> cartridge.getIO().setVoltage(voltage), cartridge.getIO()::stop, cartridge);
    }

    public Command openCartridge() {
        return Commands
                .runEnd(() -> cartridge.getIO().setVoltage(MOVING_VOLTAGE), cartridge.getIO()::stop, cartridge)
                .until(() -> cartridge.getIO().isOuterPressed());
    }

    public Command closeCartridge() {
        return Commands
                .runEnd(() -> cartridge.getIO().setVoltage(MOVING_VOLTAGE), cartridge.getIO()::stop, cartridge)
                .until(() -> cartridge.getIO().isInnerPressed());
    }

}
