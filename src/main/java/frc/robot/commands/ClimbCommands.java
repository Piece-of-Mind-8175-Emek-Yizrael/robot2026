package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.climb.Climb;
import static frc.robot.subsystem.climb.ClimbConstants.*;

import java.util.function.BooleanSupplier;

public class ClimbCommands extends Command {

    Climb climb;

    public ClimbCommands(Climb climb) {
        this.climb = climb;
    }

    public Command setKrakenVoltage(double voltage) {
        return Commands.runEnd(() -> climb.getIO().setKrakenVoltage(voltage), climb.getIO()::stopKraken, climb);
    }

    public Command stopKraken() {
        return Commands.runOnce(() -> climb.getIO().stopKraken(), climb);
    }

    public Command setNeoVoltage(double voltage) {
        return Commands.runEnd(() -> climb.getIO().setNeoVoltage(voltage), climb.getIO()::stopNeo, climb);
    }

    public Command stopNeo() {
        return Commands.runOnce(() -> climb.getIO().stopNeo(), climb);
    }

    public Command setServoAngle(double angle) {
        return Commands.sequence(
                Commands.runOnce(() -> climb.getIO().setServoAngle(angle), climb),
                new WaitCommand(SERVO_MOVE_TIME));
    }

    public Command openServo() {
        return setServoAngle(OPEN_CLIMB_SERVO_ANGLE);
    }

    public Command closeServo() {
        return setServoAngle(CLOSE_CLIMB_SERVO_ANGLE);
    }

    public Command closedUntilPressed(double voltage) {
        return Commands.runOnce(() -> climb.getIO().setKrakenVoltage(voltage), climb)
                        .until(() -> climb.getIO().isPressed());
    }

    public Command goToPos(double goal) {
        return Commands.runOnce(() -> climb.getIO().ClimbGoToPos(goal), climb)
                .until(climb.getIO()::atGoal);
    }

    public Command setClimb(double voltage) {
        return Commands.runEnd(() -> climb.getIO().setKrakenVoltage(voltage), () -> climb.getIO().stopKraken(), climb)
                .until(() -> climb.getIO().isPressed())
                .andThen(() -> climb.getIO().setServoAngle(SET_CLIMB_SERVO_ANGLE));
    }

    public Command openClimb() {
        return Commands.sequence(
                closedUntilPressed(CLOSE_CLIMB_OPEN_SERVO_VOLTAGE),
                openServo(),
                stopKraken().until(() -> climb.getIO().getPosition() >= CLIMB_OPENED_POS));
    }

    public Command openClimb(double voltage, BooleanSupplier buttonPressed) {
        return Commands.runOnce(() -> climb.getIO().setServoAngle(CLOSE_CLIMB_SERVO_ANGLE), climb)
                .withTimeout(OPEN_SWITCH_DELAY).andThen(() -> climb.getIO().setKrakenVoltage(voltage), climb)
                .until(buttonPressed).andThen(() -> climb.getIO().stopKraken(), climb);
    }// need to cheackb

    public Command climb() {
        return Commands.sequence(goToPos(CLIMB_CLOSED_POS),
                openServo(),
                stopKraken());
    }

    public Command climbDown() {
        return Commands.sequence(null);
    }


}