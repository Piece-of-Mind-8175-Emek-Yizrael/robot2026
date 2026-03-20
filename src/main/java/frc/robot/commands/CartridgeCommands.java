package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.cartridge.Cartridge;

public class CartridgeCommands extends Command {

    private Cartridge cartridge;

    public CartridgeCommands(Cartridge cartridge) {
        this.cartridge = cartridge;
    }

    public Command stop(){
        return Commands.runOnce(cartridge.getIO()::stop, cartridge);
    }

    public Command setVoltage(double voltage) {
        return Commands.runEnd(() -> cartridge.getIO().setVoltage(voltage), cartridge.getIO()::stop, cartridge);
    }

    public Command setVoltage(DoubleSupplier voltage) {
        return Commands.runEnd(() -> cartridge.getIO().setVoltage(voltage.getAsDouble()), cartridge.getIO()::stop, cartridge);
    }

    public Command openOrCloseManual(DoubleSupplier voltage) {
        DoubleSupplier volt = () -> {
            double result = Math.copySign(Math.pow(voltage.getAsDouble(), 2) * 5.0, voltage.getAsDouble());
            if(cartridge.getIO().isOuterPressed()){
                result = Math.min(result, 0);
            }
            if(cartridge.getIO().isInnerPressed()){
                result = Math.max(result, 0);
            }
            return result;
        };
        return setVoltage(volt);
    }

    public Command setOpenVoltage(double voltage) {
        return setVoltage(voltage).until(cartridge.getIO()::isOuterPressed);
    }
    
    public Command setCloseVoltage(double voltage) {
        return setVoltage(voltage).until(cartridge.getIO()::isInnerPressed);
    }

    public Command goToPosition(double postion) {
        return new FunctionalCommand(() -> cartridge.getIO().resetPID(),
                () -> cartridge.getIO().goToPos(postion), bool -> {
                    cartridge.getIO().stop();
                },
                () -> cartridge.getIO().atGoal(), cartridge);
    }

    public Command openCartridge() {
        return setOpenVoltage(2)
                .withName("open cartridge");
    }

    public Command closeCartridge() {
        return setCloseVoltage(-1.5)
                .withName("close cartridge");
    }

    public Command openAndCloseCartridge(){
        return new ConditionalCommand(
            closeCartridge(), 
            openCartridge(), 
            cartridge.getIO()::isInnerPressed);
    }

}
