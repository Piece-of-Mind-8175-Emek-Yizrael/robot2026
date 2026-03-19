package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.InterpolatorResult;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.cartridge.Cartridge;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shoot.Shoot;
import frc.robot.subsystems.shooterArm.ShooterArm;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.util.ShooterCalculator;

import static frc.robot.Commands.SwerveCommands.getHubCentricVelocity;

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

    private final double farArmAngle = 0.1;
    private final double farShootSpeed = 55.0;
    LoggedNetworkNumber tuneSpeed = new LoggedNetworkNumber("farShootSpeed", farShootSpeed);

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

    public Command intakeFuel() {
        return Commands.parallel(
                cartridgeCommands.openCartridge(),
                intakeCommands.intake());
    }

    public Command intakeFuelWithTransfer() {
        return Commands.parallel(
                cartridgeCommands.openCartridge(),
                intakeCommands.intake(),
                transferCommands.setVoltage(4));
    }

    public Command outtakeFuel() {
        return Commands.parallel(
                cartridgeCommands.openCartridge(),
                intakeCommands.outake(),
                transferCommands.setVoltage(-4));
    }

    public Command closeCartridge() {
        return Commands.parallel(
                cartridgeCommands.closeCartridge(),
                intakeCommands.intake());
    }

    public Command closeCartridgeWithTransfer() {
        return Commands.parallel(
                cartridgeCommands.closeCartridge(),
                intakeCommands.intake(),
                transferCommands.setVoltage(4));
    }

    public Command shootToHub(BooleanSupplier readyToShoot) {
        return new Command() {
            {
                addRequirements(shoot, arm, transfer);
            }

            @Override
            public void initialize() {
                shoot.getIO().setHoodSetpoint(tuneSpeed.get());// FIXME: placeholder value //55//45
            }

            @Override
            public void execute() { // TODO - uncomment when finished interpolation tuning
                double distance = swerve.getDistanceFromHub();
                shoot.getIO().setHoodSetpoint(ShooterCalculator.getTargetSpeed(distance));
                if (ShooterCalculator.isFar(distance)) {
                arm.getIO().setVoltage(1);
                } else {
                arm.getIO().stopMotor();
                }

                if (readyToShoot.getAsBoolean()) {
                    shoot.getIO().setFeedVoltage(8.0);
                    transfer.getIO().setVoltage(5.0);
                } else {
                    shoot.getIO().stopFeed();
                    transfer.getIO().stopMotor();
                }
                if (ShooterCalculator.isFar(distance)) {
                arm.getIO().setVoltage(1);
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

    public Command shootToHubInMovement(BooleanSupplier readyToShoot) {
        return new Command() {
            {
                addRequirements(shoot, arm, transfer);
            }

            @Override
            public void initialize() {
                shoot.getIO().setHoodSetpoint(tuneSpeed.get());// FIXME: placeholder value //55//45
            }

            @Override
            public void execute() { // TODO - uncomment when finished interpolation tuning
                double distance = swerve.getDistanceFromHub();

                Translation2d velocity = getHubCentricVelocity(swerve);
                InterpolatorResult result = ShooterCalculator.getTargetSpeedAndRotation(distance, velocity.getX(), velocity.getY()); // TODO: is this the correct order?

                shoot.getIO().setHoodSetpoint(ShooterCalculator.getTargetSpeed(result.speed()));

                if (ShooterCalculator.isFar(distance)) {
                arm.getIO().setVoltage(1);
                } else {
                arm.getIO().stopMotor();
                }

                if (readyToShoot.getAsBoolean()) {
                    shoot.getIO().setFeedVoltage(8.0);
                    transfer.getIO().setVoltage(5.0);
                } else {
                    shoot.getIO().stopFeed();
                    transfer.getIO().stopMotor();
                }
                if (ShooterCalculator.isFar(distance)) {
                arm.getIO().setVoltage(1);
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
    
}
