package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.cartridge.Cartridge;
import static frc.robot.subsystems.cartridge.CartridgeConstants.*;

public class CartridgeCommands extends Command {

    private Cartridge cartridge;

    public CartridgeCommands(Cartridge cartridge) {
        this.cartridge = cartridge;
    }

    public Command setVoltage(double voltage) {
        return Commands.runEnd(() -> cartridge.getIO().setVoltage(voltage), cartridge.getIO()::stop, cartridge);
    }

    public Command setOpenVoltage() {
        return setVoltage(3).until(cartridge.getIO()::isOuterPressed);
    }

    public Command setCloseVoltage() {
        return setVoltage(-4).until(cartridge.getIO()::isInnerPressed);
    }

    public Command goToPosition(double postion) {
        return new FunctionalCommand(() -> cartridge.getIO().resetPID(),
                () -> cartridge.getIO().goToPos(postion), bool -> {
                    cartridge.getIO().stop();
                },
                () -> cartridge.getIO().atGoal(), cartridge);
    }

    public Command openCartridge() {
        return goToPosition(OPEN_CARTRIDGE_POS).andThen(setOpenVoltage()).withName("open cartridge");
    }

    public Command closeCartridge() {
        return goToPosition(CLOSE_CARTRIDGE_POS).andThen(setCloseVoltage().withName("close cartridge"));
    }

    // public Command openAndCloseCartridge(){

    // }

}
