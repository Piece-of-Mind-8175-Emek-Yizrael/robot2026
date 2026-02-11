package frc.robot.commands;

import static frc.robot.subsystems.shoot.ShootConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shoot.Shoot;

public class ShootCommands {

    private final Shoot shoot;

    public ShootCommands(Shoot shoot){
        this.shoot = shoot;
    }
    public Command setVoltage(double hoodVoltage, double feedVoltage) {
        return Commands.runEnd(
            () -> shoot.getIO().setBoth(hoodVoltage, feedVoltage),
            shoot.getIO()::stopBoth,
            shoot
            ).withName("set voltage both");
    }

    public Command stopBoth() {
        return Commands.runOnce(() -> shoot.getIO().stopBoth(), shoot).withName("stop both");
    }

    public Command setHoodGoal(double goal) {
        return new Command() {
            private boolean feedStarted = false;
            
            {
                addRequirements(shoot);
                setName("set voltage both command");
            }

            @Override
            public void initialize() {
                shoot.getIO().setHoodSetpoint(goal);
            }

            @Override
            public void execute() {
                if(shoot.getIO().atGoal() && !feedStarted) {
                    shoot.getIO().setFeedVoltage(FEED_SHOOT_VOLTAGE);
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
