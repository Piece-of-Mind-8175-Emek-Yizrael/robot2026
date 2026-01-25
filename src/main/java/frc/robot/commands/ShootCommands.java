package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shoot.Shoot;

public class ShootCommands {

    public Command setVoltage(double hoodVoltage, double transferVoltage, Shoot shoot) {
        return Commands.runEnd(() -> shoot.getIO().setVoltage(hoodVoltage, transferVoltage),
                                shoot.getIO()::stopBoth,
                                shoot);
    }

    public Command stopBoth(Shoot shoot) {
        return Commands.runOnce(() -> shoot.getIO().stopBoth(), shoot);
    }
    
}
