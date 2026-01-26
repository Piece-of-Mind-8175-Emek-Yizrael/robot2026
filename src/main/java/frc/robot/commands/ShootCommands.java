package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shoot.Shoot;

public class ShootCommands {

    public Command setVoltage(double hoodVoltage, double feedVoltage, Shoot shoot) {
        return Commands.runEnd(() -> shoot.getIO().setBoth(hoodVoltage, feedVoltage),
                                shoot.getIO()::stopBoth,
                                shoot);
    }

    public Command stopBoth(Shoot shoot) {
        return Commands.runOnce(() -> shoot.getIO().stopBoth(), shoot);
    }

    public Command setHoodGoal(double goal, Shoot shoot) {
        return new Command() {
            private boolean feedStarted = false;
            
            {
                addRequirements(shoot);
            }

            @Override
            public void initialize() {
                addRequirements(shoot);
                shoot.getIO().setGoal(goal);
            }

            @Override
            public void execute() {
                if(shoot.getIO().atGoal() && !feedStarted) {
                    shoot.getIO().setFeedVoltage(4);
                    feedStarted = true;
                }
            }

            @Override
            public void end(boolean interrupted) {
                shoot.getIO().stopBoth();
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }
    
}
