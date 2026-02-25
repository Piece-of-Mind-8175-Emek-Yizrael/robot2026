package frc.robot.commands;

import static frc.robot.subsystems.shooterArm.ShooterArmConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooterArm.ShooterArm;

public class ShooterArmCommands {
    private ShooterArm arm;

    public ShooterArmCommands(ShooterArm arm){
        this.arm = arm;
    }

    public Command setVoltage(double voltage) {
        return Commands.startEnd(() -> arm.getIO().setVoltage(voltage),
                arm.getIO()::stopMotor, arm);
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
                arm.getIO().stopMotor();// TODO decide if we want to stop motor or hold position
            }

            @Override
            public boolean isFinished() {
                return arm.getIO().atGoal();
            }
        };
    }

    public Command closeArm() {
        return goToPosition(CLOSE_POS);
    }

    public Command openArm() {
        return goToPosition(OPEN_POS);
    }

    public Command ressistGravity() {
        return Commands.run(() -> arm.getIO().resistGravity(), arm);
    }
    
    public Command resetPos() {
        return Commands.run(arm.getIO()::resetIfPrees, arm);
    }

}
