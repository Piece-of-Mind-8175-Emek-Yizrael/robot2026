package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.cartridge.cartridge;
import static frc.robot.subsystems.cartridge.cartridgeConstants.*;

public class cartridgeCommands extends Command {

    cartridge cartridge;

    public cartridgeCommands(cartridge cartridge) {
        this.cartridge = cartridge;
    }

    public Command setVoltage(double voltage) {
        return Commands.runEnd(() -> cartridge.getIO().setVoltage(voltage), () -> cartridge.getIO().stop(), cartridge);
    }

    public Command openCartridgeClosedLoop() {
        return Commands.runEnd(() -> cartridge.getIO().goToPosition(OPEN_POS), () -> cartridge.getIO().stop(),
                cartridge);
    }

    public Command closeCartridgeOpenLoop() {
        return Commands
                .runEnd(() -> cartridge.getIO().setVoltage(CLOSING_VOLTAGE), () -> cartridge.getIO().stop(), cartridge)
                .until(() -> cartridge.getIO().isPressed());
    }

    public Command closeCartridgeClosedLoop() {
        return Commands
                .runEnd(() -> cartridge.getIO().goToPosition(0),
                        () -> cartridge.getIO().setVoltage(SLOW_CLOSING_VOLTAGE), cartridge)
                .until(() -> cartridge.getIO().isPressed()).andThen(() -> cartridge.getIO().stop());
    }

}
