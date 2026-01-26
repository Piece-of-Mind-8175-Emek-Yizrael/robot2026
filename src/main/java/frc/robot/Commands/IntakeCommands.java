package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake.*;

public class IntakeCommands {

    private final CommandXboxController controller;

    public IntakeCommands(CommandXboxController controller) {
        this.controller = controller;
    }

    public Command intake(Intake intake) {
        return Commands.runEnd(() -> intake.getIO().setVoltage(5) ,
        () -> intake.getIO().setVoltage(0), 
        intake);
    }
}