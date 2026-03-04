package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.cartridge.Cartridge;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shoot.Shoot;
import frc.robot.subsystems.shooterArm.ShooterArm;
import frc.robot.subsystems.transfer.Transfer;

public class SuperCommands {

    //subsystems
    Cartridge cartridge;
    Intake intake;
    Shoot shoot;
    ShooterArm arm;
    Transfer transfer;

    //commands
    CartridgeCommands cartridgeCommands;
    IntakeCommands intakeCommands;
    ShootCommands shootCommands;
    ShooterArmCommands armCommands;
    TransferCommands transferCommands;

    public SuperCommands(Cartridge cartridge, Intake intake, Shoot shoot, ShooterArm arm, Transfer transfer) {
        this.cartridge = cartridge;
        this.intake = intake;
        this.shoot = shoot;
        this.arm = arm;
        this.transfer = transfer;

        this.cartridgeCommands = new CartridgeCommands(cartridge);
        this.intakeCommands = new IntakeCommands(intake);
        this.shootCommands = new ShootCommands(shoot);
        this.armCommands = new ShooterArmCommands(arm);
        this.transferCommands = new TransferCommands(transfer);
    }

    public Command intakeFuel(){
        return Commands.parallel(
            cartridgeCommands.openCartridge(),
            intakeCommands.intake()
        );
    }

    public Command shootToHub(boolean readyToShoot) {
        return new Command() {
            {
                addRequirements(intake, shoot, arm, transfer);
            }

            @Override
            public void initialize() {
                shoot.getIO().setHoodSetpoint(50);//FIXME: placeholder value
                arm.getIO().setGoal(0);//FIXME: placeholder value
            }

            @Override
            public void execute() {
                if(readyToShoot){
                    shoot.getIO().setFeedVoltage(12.0);
                    transfer.getIO().setVoltage(0.5);
                    intake.getIO().setVoltage(3);
                }
                
            }

            @Override
            public void end(boolean interrupted) {
                shoot.getIO().stopBoth();
                transfer.getIO().stopMotor();
                intake.getIO().stopMotor();
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }
}
