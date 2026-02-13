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

    public Command setVoltage(double voltage) {
        return Commands.runEnd(() -> climb.getIO().setMotorVoltage(voltage), climb.getIO()::stopMotor, climb);
    }

    public Command stopMotor() {
        return Commands.runOnce(() -> climb.getIO().stopMotor(), climb);
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
        return Commands.runOnce(() -> climb.getIO().setMotorVoltage(voltage), climb)
                .andThen(Commands.waitUntil(climb.getIO()::isPressed));
    }

    public Command goToPos(double goal) {
        return Commands.runOnce(() -> climb.getIO().ClimbGoToPos(goal), climb)
                .andThen(Commands.waitUntil(climb.getIO()::atGoal));
    }

    public Command setClimb(double voltage) {
        return Commands.runEnd(() -> climb.getIO().setMotorVoltage(voltage), () -> climb.getIO().stopMotor(), climb)
                .until(() -> climb.getIO().isPressed())
                .andThen(() -> climb.getIO().setServoAngle(SET_CLIMB_SERVO_ANGLE));
    }

    public Command openClimb() {
        return Commands.sequence(
                closedUntilPressed(CLOSE_CLIMB_OPEN_SERVO_VOLTAGE),
                openServo(),
                stopMotor().until(() -> climb.getIO().getPosition() >= CLIMB_OPENED_POS));
    }

    public Command openClimb(double voltage, BooleanSupplier buttonPressed) {
        return Commands.runOnce(() -> climb.getIO().setServoAngle(CLOSE_CLIMB_SERVO_ANGLE), climb)
                .withTimeout(OPEN_SWITCH_DELAY).andThen(() -> climb.getIO().setMotorVoltage(voltage), climb)
                .until(buttonPressed).andThen(() -> climb.getIO().stopMotor(), climb);
    }// need to cheackb

    public Command climb() {
        return Commands.sequence(goToPos(CLIMB_CLOSED_POS),
                openServo(),
                stopMotor());
    }

    public Command climbDown() {
        return Commands.sequence(null);
    }

    // public Command setPreClimb() {
    // return new FunctionalCommand(() ->
    // climb.getIO().setServoAngle(OPEN_CLIMB_SERVO_ANGLE),
    // () -> climb.getIO().preClimbGoToPos(PRE_CLIMB_POS),
    // interrupted -> climb.getIO().stopMotor(),
    // () -> climb.getIO().atGoal(), climb);
    // }

    // public Command setClimb() {
    // return new FunctionalCommand(null, () -> climb.getIO().ClimbGoToPos(),
    // interrupted -> {
    // climb.getIO().setServoAngle(CLOSE_CLIMB_SERVO_ANGLE);
    // climb.getIO().stopMotor();
    // }, () -> climb.getIO().atGoal(), climb);
    // }

}