package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shoot.Shoot;
import static frc.robot.subsystems.shoot.ShootConstants.*;

public class ShootCommands {

    private final Shoot shoot;

    public ShootCommands(Shoot shoot) {
        this.shoot = shoot;
    }

    public Command setVoltage(double hoodVoltage, double feedVoltage) {
        return Commands.runEnd(
                () -> shoot.getIO().setBoth(hoodVoltage, feedVoltage),
                shoot.getIO()::stopBoth,
                shoot).withName("set voltage both");
    }

    public Command setHoodVoltage() {
        double hoodVoltage = 5.5;
        return Commands.runEnd(
                () -> shoot.getIO().setHoodVoltage(hoodVoltage),
                shoot.getIO()::stopHood,
                shoot).withName("set hood voltage");
    }

    public Command setFeedVoltage() {
        double feedVoltage = 8.0;
        return Commands.runEnd(
                () -> shoot.getIO().setFeedVoltage(feedVoltage),
                shoot.getIO()::stopFeed,
                shoot).withName("set feed voltage");
    }

    public Command stopBoth() {
        return Commands.runOnce(() -> shoot.getIO().stopBoth(), shoot).withName("stop both");
    }

    public Command setSetHoodVelocity(double setPoint) {
        return Commands.runEnd(
                () -> shoot.getIO().setHoodSetpoint(setPoint),
                shoot.getIO()::stopHood,
                shoot);
    }

    public Command setSetFeedVelocity(double setPoint) {
        return Commands.runEnd(
                () -> shoot.getIO().setFeedSetpoint(setPoint),
                shoot.getIO()::stopFeed,
                shoot);
    }

    public Command shootBoth(double goal) {
        return new Command() {
            private boolean feedStarted = false;

            {
                addRequirements(shoot);
                setName("set voltage both command");
            }

            @Override
            public void initialize() {
                feedStarted = false;
                shoot.getIO().setHoodSetpoint(goal);
            }

            @Override
            public void execute() {
                if (shoot.getIO().atGoalHood() && !feedStarted) {
                    shoot.getIO().setFeedVoltage(feedVol);

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
        }.andThen(Commands.runOnce(shoot.getIO()::resetCommand, shoot));
    }

    public Command setHoodGoal(double goal) {
        return new Command() {
            {
                addRequirements(shoot);
                setName("set hood goal command");
            }

            @Override
            public void initialize() {
                shoot.getIO().setHoodSetpoint(goal);
            }

            @Override
            public void end(boolean interrupted) {
                shoot.getIO().stopHood();
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }
}
