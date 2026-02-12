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

    public Command setSetHoodVelocity(double setPoint) {
        return Commands.runEnd(
            () -> shoot.getIO().setHoodSetpoint(setPoint),
            shoot.getIO()::stopHood,
            shoot
            );
    }
    
    public Command setSetFeedVelocity(double setPoint) {
        return Commands.runEnd(
            () -> shoot.getIO().setFeedSetpoint(setPoint),
            shoot.getIO()::stopFeed,
            shoot
            );
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
                if(shoot.getIO().atGoalHood() && !feedStarted) {
                    shoot.getIO().setFeedSetpoint(FEED_SHOOT_SETPOINT);
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
