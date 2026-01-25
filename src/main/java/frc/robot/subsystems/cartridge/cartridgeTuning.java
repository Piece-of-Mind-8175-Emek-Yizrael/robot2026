package frc.robot.subsystems.cartridge;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import static frc.robot.subsystems.cartridge.cartridgeConstants.*;

public class cartridgeTuning {

    LoggedNetworkNumber KpTune = new LoggedNetworkNumber("kP", kP);
    LoggedNetworkNumber KdTune = new LoggedNetworkNumber("kD", kD);
    LoggedNetworkNumber KiTune = new LoggedNetworkNumber("kI", kI);
    LoggedNetworkNumber KsTune = new LoggedNetworkNumber("kS", kS);
    LoggedNetworkNumber KvTune = new LoggedNetworkNumber("kV", kV);

    public double getKp() {
        return KpTune.get();
    }

    public double getKi() {
        return KiTune.get();
    }

    public double getKd() {
        return KdTune.get();
    }

    public double getKs() {
        return KsTune.get();
    }

    public double getKv() {
        return KvTune.get();
    }

}
