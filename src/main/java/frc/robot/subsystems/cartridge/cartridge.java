package frc.robot.subsystems.cartridge;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class cartridge extends SubsystemBase {
    cartridgeIOInputsAutoLogged inputs = new cartridgeIOInputsAutoLogged();
    cartridgeIO io;

    public cartridge(cartridgeIO io) {
        this.io = io;
    }

    public cartridgeIO getIO() {
        return io;
    }

    @Override
    public void periodic() {
        Logger.processInputs("cartridge", inputs);
    }
}
