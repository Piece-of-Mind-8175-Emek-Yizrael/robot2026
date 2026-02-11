package frc.robot.Commands;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_VOLTS;
import static frc.robot.subsystems.intake.IntakeConstants.OUTAKE_VOLTS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.*;

public class IntakeCommands {

    private final Intake intake;

    public IntakeCommands(Intake intake) {
        this.intake = intake;
    }

    public Command intake() {
        return Commands.runEnd(
            () -> intake.getIO().setVoltage(INTAKE_VOLTS) ,
            () -> intake.getIO().stopMotor(), 
            intake);
    }

    public Command outake(){
        return Commands.runEnd(
            () -> intake.getIO().setVoltage(OUTAKE_VOLTS),
            () -> intake.getIO().stopMotor(),
            intake);
    }
}