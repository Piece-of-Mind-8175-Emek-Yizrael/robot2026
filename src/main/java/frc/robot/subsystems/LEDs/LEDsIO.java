package frc.robot.subsystems.LEDs;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public interface LEDsIO {
    @AutoLog
    public static class LEDsIOInputs {
        public String[] ledColorList;
    }

    public default void updateInputs(LEDsIOInputs inputs) {
    }

    public default void setAll(Color color) {
    }

    public default void setParts(Color... colors) {
    }

    public default void blink(Color color, double seconds) {
    }

    public default void blink(LEDPattern pattern, double seconds) {
    }

    public default void rainbow() {
    }

}
