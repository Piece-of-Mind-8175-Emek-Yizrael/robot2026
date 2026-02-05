package frc.robot.subsystems.cartridge;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import static frc.robot.subsystems.cartridge.CartridgeConstants.*;

public class CartridgeTuning {

    LoggedNetworkNumber KpTune = new LoggedNetworkNumber("Kp", Kp);
    LoggedNetworkNumber KdTune = new LoggedNetworkNumber("Kd", Kd);
    LoggedNetworkNumber KiTune = new LoggedNetworkNumber("Ki", Ki);
    LoggedNetworkNumber MaxVelocityTune = new LoggedNetworkNumber("max velocity", MAX_VELOCITY);
    LoggedNetworkNumber MaxAccelerationTune = new LoggedNetworkNumber("max acceleration", MAX_ACCELERATION);

    public double getKp() {
        return KpTune.get();
    }

    public double getKi() {
        return KiTune.get();
    }

    public double getKd() {
        return KdTune.get();
    }

    public double getMaxVelocity() {
        return MaxVelocityTune.get();
    }

    public double getMaxAcceleration() {
        return MaxAccelerationTune.get();
    }

}
