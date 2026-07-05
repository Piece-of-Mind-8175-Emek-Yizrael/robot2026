package frc.robot.Commands;

import static frc.robot.subsystems.cartridge.CartridgeConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.cartridge.Cartridge;

public class CartridgeCommands extends Command {

    private Cartridge cartridge;

    public CartridgeCommands(Cartridge cartridge) {
        this.cartridge = cartridge;
    }

    public Command stop() {
        return Commands.runOnce(cartridge.getIO()::stop, cartridge);
    }

    public Command setVoltage(double voltage) {
        return Commands.runEnd(() -> cartridge.getIO().setVoltage(voltage), cartridge.getIO()::stop, cartridge);
    }

    public Command setOpenVoltage(double voltage) {
        return setVoltage(voltage).until(cartridge.getIO()::isOpenPressed);
    }

    public Command setCloseVoltage(double voltage) {
        return setVoltage(voltage).until(cartridge.getIO()::isClosePressed);
    }

    public Command openCartridge() {
        return setOpenVoltage(openCartridgeVolt)
                .withName("open cartridge");
    }

    public Command closeCartridge() {
        return setCloseVoltage(closeCartridgeVolt)
                .withName("close cartridge");
    }

    public Command shakeCartridge() {
        return Commands.sequence(
                setVoltage(shakeCloseCartridgeVolt).withTimeout(shakeTimeOut),
                setVoltage(shakeOpenCartridgeVolt).withTimeout(shakeTimeOut))
                .repeatedly()
                .withName("Continuous Shake");
    }

}
