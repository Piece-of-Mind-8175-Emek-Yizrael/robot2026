package frc.robot.commands;

import static frc.robot.subsystems.shooterArm.ShooterArmConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooterArm.ShooterArm;
import frc.robot.subsystems.shooterArm.ShooterArmIO;

public class ShooterArmCommands {
    
    public Command setVoltage(double voltage, ShooterArm arm) {
        return Commands.startEnd(() -> arm.getIO().setVoltage(voltage),
                                arm.getIO()::stopMotor, arm);
    }

    public Command goToPosition(double position, ShooterArm arm) {
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
                arm.getIO().stopMotor();//TODO decide if we want to stop motor or hold position
            }

            @Override
            public boolean isFinished() {
                return arm.getIO().atGoal();
            }
        };
    }

    public Command closeArm(ShooterArm arm) {
        return goToPosition(CLOSE_POS, arm);
    }

    public Command openArm(ShooterArm arm) {
        return goToPosition(OPEN_POS, arm);
    }
}
