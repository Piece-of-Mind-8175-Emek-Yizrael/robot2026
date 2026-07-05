package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.cartridge.Cartridge;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.*;
import frc.robot.subsystems.shoot.Shoot;
import static frc.robot.subsystems.shoot.ShootConstants.*;
import frc.robot.subsystems.shooterArm.ShooterArm;
import static frc.robot.subsystems.shooterArm.ShooterArmConstants.*;
import frc.robot.subsystems.transfer.Transfer;
import static frc.robot.subsystems.transfer.TransferConstants.*;
import frc.robot.util.ShooterCalculator;

public class SuperCommands {

    // subsystems
    private Cartridge cartridge;
    private Intake intake;
    private Shoot shoot;
    private ShooterArm arm;
    private Transfer transfer;
    private Swerve swerve;

    // commands

    private CartridgeCommands cartridgeCommands;
    private IntakeCommands intakeCommands;
    private ShootCommands shootCommands;
    private ShooterArmCommands armCommands;
    private TransferCommands transferCommands;
    private SwerveCommands swerveCommands;
    private Timer timer = new Timer();

    private boolean readyToShoot = false;
    private double hubDistance = 3.0;

    public SuperCommands(Cartridge cartridge, Intake intake, Shoot shoot, ShooterArm arm, Transfer transfer,
            Swerve swerve) {
        this.cartridge = cartridge;
        this.intake = intake;
        this.shoot = shoot;
        this.arm = arm;
        this.transfer = transfer;
        this.swerve = swerve;

        this.cartridgeCommands = new CartridgeCommands(cartridge);
        this.intakeCommands = new IntakeCommands(intake);
        this.shootCommands = new ShootCommands(shoot);
        this.armCommands = new ShooterArmCommands(arm);
        this.transferCommands = new TransferCommands(transfer);
        shootCommands = new ShootCommands(shoot);

    }

    public Command intakeFuelWithOutTransfer() {
        return Commands.parallel(
                cartridgeCommands.openCartridge(),
                intakeCommands.intake());
    }

    public Command intakeFuelWithTransfer() {
        return Commands.parallel(
                cartridgeCommands.openCartridge(),
                intakeCommands.intake(),
                transferCommands.setVoltage(transferIntakeVolt));
    }

    public Command intakeFuel(BooleanSupplier isShooting) {
        return new ConditionalCommand(
                intakeFuelWithOutTransfer(),
                intakeFuelWithTransfer(),
                isShooting);
    }

    public Command outtakeFuel() {
        return Commands.parallel(
                cartridgeCommands.openCartridge(),
                intakeCommands.outake(),
                transferCommands.setVoltage(transferOuttakeVolt));
    }

    public Command closeCartridge() {
        return Commands.parallel(
                cartridgeCommands.closeCartridge(),
                intakeCommands.intake());
    }

    public Command shootToHub(BooleanSupplier readyToShoot) {
        return new Command() {
            {
                addRequirements(shoot, arm, transfer);
            }

            @Override
            public void initialize() {
                shoot.getIO().setHoodSetpoint(ShooterCalculator.getTargetSpeed(swerve.getDistanceFromHub()));
            }

            @Override
            public void execute() {
                double distance = swerve.getDistanceFromHub();
                shoot.getIO().setHoodSetpoint(ShooterCalculator.getTargetSpeed(distance));
                if (ShooterCalculator.isFar(distance)) {
                    arm.getIO().setVoltage(openArmVolt);
                } else {
                    arm.getIO().stopMotor();
                }

                if (readyToShoot.getAsBoolean()) {
                    shoot.getIO().setFeedVoltage(feedVol);
                    transfer.getIO().setVoltage(transferIntakeShotterVolt);
                    intake.getIO().setVoltage(8.0);
                } else {
                    shoot.getIO().stopFeed();
                    transfer.getIO().stopMotor();
                    intake.getIO().stopMotor();
                }
                if (ShooterCalculator.isFar(distance)) {
                    arm.getIO().setVoltage(openArmVolt);
                } else {
                    arm.getIO().stopMotor();
                }

            }

            @Override
            public void end(boolean interrupted) {
                shoot.getIO().stopBoth();
                transfer.getIO().stopMotor();
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    public Command autoPreShootToHub() {
        return new Command() {
            {
                addRequirements(shoot, arm);
            }

            @Override
            public void initialize() {
                shoot.getIO().setHoodSetpoint(ShooterCalculator.getTargetSpeed(swerve.getDistanceFromHub()));
                readyToShoot = false;
            }

            @Override
            public void execute() {
                double distance = swerve.getDistanceFromHub();
                shoot.getIO().setHoodSetpoint(ShooterCalculator.getTargetSpeed(distance));
                if (ShooterCalculator.isFar(distance)) {
                    arm.getIO().setVoltage(openArmVolt);
                } else {
                    arm.getIO().stopMotor();
                }
                if (ShooterCalculator.isFar(distance)) {
                    arm.getIO().setVoltage(openArmVolt);
                } else {
                    arm.getIO().stopMotor();
                }

            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    public Command autoShootToHub() {
        readyToShoot = false;
        return new Command() {
            {
                addRequirements(shoot, arm, transfer);
            }

            @Override
            public void initialize() {
                shoot.getIO().setHoodSetpoint(swerve.getDistanceFromHub());
                readyToShoot = false;
            }

            @Override
            public void execute() {
                double distance = swerve.getDistanceFromHub();
                shoot.getIO().setHoodSetpoint(ShooterCalculator.getTargetSpeed(distance));

                shoot.getIO().setFeedVoltage(feedVol);
                transfer.getIO().setVoltage(transferIntakeVolt);

                if (ShooterCalculator.isFar(distance)) {
                    arm.getIO().setVoltage(openArmVolt);
                } else {
                    arm.getIO().stopMotor();
                }
                if (ShooterCalculator.isFar(distance)) {
                    arm.getIO().setVoltage(openArmVolt);
                } else {
                    arm.getIO().stopMotor();
                }

            }

            @Override
            public void end(boolean interrupted) {
                shoot.getIO().stopBoth();
                transfer.getIO().stopMotor();
                readyToShoot = false;
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    public Command shakeCartridge() {
        return Commands.parallel(
                cartridgeCommands.shakeCartridge(),
                intakeCommands.intake());
    }

}
