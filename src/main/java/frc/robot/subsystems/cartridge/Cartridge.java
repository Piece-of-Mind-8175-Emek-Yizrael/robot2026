package frc.robot.subsystems.cartridge;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Cartridge extends SubsystemBase {
    private final CartridgeIOInputsAutoLogged inputs = new CartridgeIOInputsAutoLogged();
    private CartridgeIO io;

    public Cartridge(CartridgeIO io) {
        this.io = io;
    }

    public CartridgeIO getIO() {
        return io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("cartridge", inputs);
    }

}
