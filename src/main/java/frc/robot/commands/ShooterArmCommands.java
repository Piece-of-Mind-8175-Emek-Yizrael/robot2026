package frc.robot.Commands;

import static frc.robot.subsystems.shooterArm.ShooterArmConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooterArm.ShooterArm;

public class ShooterArmCommands {
    private ShooterArm arm;

    public ShooterArmCommands(ShooterArm arm) {
        this.arm = arm;
    }

    public Command setVoltage(double voltage) {
        return Commands.startEnd(() -> arm.getIO().setVoltage(voltage),
                arm.getIO()::stopMotor, arm);
    }

    public Command stopArm() {
        return setVoltage(0);
    }

    public Command goToPosition(double position) {
        return new Command() {
            @Override
            public void initialize() {
                arm.getIO().resetPID(position);
            }

            @Override
            public void execute() {
                arm.getIO().setGoal(position);
            }

            @Override
            public void end(boolean interrupted) {
                arm.getIO().stopMotor();
            }

            @Override
            public boolean isFinished() {
                return arm.getIO().atGoal();
            }
        };
    }

    public Command closeArmManual() {
        return setVoltage(closeArmVolt)
                .until(arm.getIO()::getSensor);
    }

    public Command openArmManual() {
        return setVoltage(openArmVolt);
    }

}
