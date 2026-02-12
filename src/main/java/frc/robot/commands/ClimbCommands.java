package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystem.climb.Climb;
import static frc.robot.subsystem.climb.ClimbConstants.*;

import java.util.function.BooleanSupplier;

public class ClimbCommands extends Command {

    Climb climb;

    public ClimbCommands(Climb climb) {
        this.climb = climb;
    }

    public Command setVoltage(double voltage) {
        return Commands.runEnd(() -> climb.getIO().setClimbVoltage(voltage), () -> climb.getIO().stopClimb(), climb);
    }

    public Command setServoAngle(double angle) {
        return Commands.runOnce(() -> climb.getIO().setServoAngle(CLOSE_CLIMB_SERVO_ANGLE), climb);
    }

    public Command setClimb(double voltage) {
        return Commands.runEnd(() -> climb.getIO().setClimbVoltage(voltage), () -> climb.getIO().stopClimb(), climb)
                .until(() -> climb.getIO().isPressed())
                .andThen(() -> climb.getIO().setServoAngle(SET_CLIMB_SERVO_ANGLE));
    }

    public Command openClimb(double voltage, BooleanSupplier buttonPressed) {
        return Commands.runOnce(() -> climb.getIO().setServoAngle(CLOSE_CLIMB_SERVO_ANGLE), climb)
                .withTimeout(OPEN_SWITCH_DELAY).andThen(() -> climb.getIO().setClimbVoltage(voltage), climb)
                .until(buttonPressed).andThen(() -> climb.getIO().stopClimb(), climb);// need to cheack

    }
}
