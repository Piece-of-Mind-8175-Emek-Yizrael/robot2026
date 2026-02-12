package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.LEDs.LEDs;

public class LEDsCommands {

    public static Command setAll(LEDs leds, Color color) {
        return Commands.run(() -> leds.setAll(color), leds);
    }

    public static Command setParts(LEDs leds, Color... colors) {
        return Commands.run(() -> leds.setParts(colors), leds);
    }

    public static Command blink(LEDs leds, Color color, double seconds) {
        return Commands.run(() -> leds.blink(color, seconds), leds);
    }

    public static Command blink(LEDs leds, LEDPattern pattern, double seconds) {
        return Commands.run(() -> leds.blink(pattern, seconds), leds);
    }

    public static Command rainbow(LEDs leds) {
        return Commands.run(() -> leds.rainbow(), leds);

    }
}