package frc.robot.subsystems.LEDs;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

    private final LEDsIO ledsIO;
    private final LEDsIOInputsAutoLogged inputs = new LEDsIOInputsAutoLogged();

    public LEDs(LEDsIO ledsIO) {
        this.ledsIO = ledsIO;
    }

    public void setAll(Color color) {
        ledsIO.setAll(color);
    }

    public void setParts(Color... colors) {
        ledsIO.setParts(colors);
    }

    public void blink(Color color, double seconds) {
        ledsIO.blink(color, seconds);
    }

    public void blink(LEDPattern pattern, double seconds) {
        ledsIO.blink(pattern, seconds);
    }

    public void rainbow() {
        ledsIO.rainbow();
    }

    @Override
    public void periodic() {
        ledsIO.updateInputs(inputs);
        Logger.processInputs("LEDs", inputs);
    }

}