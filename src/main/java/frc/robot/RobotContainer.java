// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.CartridgeCommands;
import frc.robot.Commands.IntakeCommands;
import frc.robot.Commands.LEDsCommands;
import frc.robot.Commands.ShootCommands;
import frc.robot.Commands.ShooterArmCommands;
import frc.robot.Commands.SuperCommands;
import frc.robot.Commands.SwerveCommands;
import frc.robot.Commands.TransferCommands;
import frc.robot.POM_lib.Joysticks.PomXboxController;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.LEDs.LEDsIOReal;
import frc.robot.subsystems.cartridge.Cartridge;
import frc.robot.subsystems.cartridge.CartridgeIOTalon;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOReal;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.shoot.Shoot;
import frc.robot.subsystems.shoot.ShootIOReal;
import frc.robot.subsystems.shooterArm.ShooterArm;
import frc.robot.subsystems.shooterArm.ShooterArmIOReal;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferIOReal;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.Apriltag.ApriltagVisionIOReal;
import frc.robot.util.ShooterCalculator;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here. :)
 */

public class RobotContainer {
        // Subsystems
        private final Swerve swerve;
        private final Intake intake;
        private final Shoot shoot;
        private final Transfer transfer;
        private final Cartridge cartridge;
        private final ShooterArm arm;
        private final VisionSubsystem vision;
        private final SuperCommands superCommands;
        private final LEDs leds;

        // Commands
        private final IntakeCommands intakeCommands;
        private final ShootCommands shootCommands;
        private final TransferCommands transferCommands;
        private final CartridgeCommands cartridgeCommands;
        private final ShooterArmCommands armCommands;

        // Controller
        private final PomXboxController driverController = new PomXboxController(0);

        private final CommandPS5Controller operatorController = new CommandPS5Controller(1);

        // Dashboard inputs
        private final SendableChooser<Command> autoChooser;

        private SwerveDriveSimulation driveSimulation = null;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                ShooterCalculator.init();
                System.out.println("Current mode: " + Constants.currentMode);
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations
                                // driveSimulation = null;
                                swerve = new Swerve(new GyroIOPigeon2(), new ModuleIOReal(0), new ModuleIOReal(1),
                                                new ModuleIOReal(2), new ModuleIOReal(3));
                                intake = new Intake(new IntakeIOReal());
                                shoot = new Shoot(new ShootIOReal());
                                transfer = new Transfer(new TransferIOReal());
                                cartridge = new Cartridge(new CartridgeIOTalon());
                                arm = new ShooterArm(new ShooterArmIOReal());
                                leds = new LEDs(new LEDsIOReal());

                                ApriltagVisionIOReal[] cameras = {
                                                new ApriltagVisionIOReal("first_camera",
                                                        VisionConstants.InitialRobotToShooterCameraTranslation),
                                                // new ApriltagVisionIOReal("seconde_camera",
                                                //                 VisionConstants.InitialRobotToShooterCameraTranslation),
                                };

                                vision = new VisionSubsystem(swerve::addVisionMeasurement, cameras, null,
                                                cartridge.getIO()::getCartridgePose, Optional.empty());

                                superCommands = new SuperCommands(cartridge, intake, shoot, arm, transfer,
                                                swerve);

                                intakeCommands = new IntakeCommands(intake);
                                shootCommands = new ShootCommands(shoot);
                                transferCommands = new TransferCommands(transfer);
                                cartridgeCommands = new CartridgeCommands(cartridge);
                                armCommands = new ShooterArmCommands(arm);

                                NamedCommands.registerCommand("intakeFuel", superCommands.intakeFuelWithTransfer());
                                NamedCommands.registerCommand("stopIntake", intakeCommands.stopIntake());
                                NamedCommands.registerCommand("preShootToHub", superCommands.autoPreShootToHub());
                                NamedCommands.registerCommand("shootToHub", superCommands.autoShootToHub());
                                NamedCommands.registerCommand("turnToHub", SwerveCommands.turnToHub(swerve));
                                NamedCommands.registerCommand("openCartridge", cartridgeCommands.openCartridge());
                                NamedCommands.registerCommand("shakeCartridge", superCommands.shakeCartridge());
                                NamedCommands.registerCommand("driveIntakeSlow", SwerveCommands
                                                .joystickDriveRobotRelative(swerve, () -> -0.2, () -> 0, () -> 0));

                                break;

                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations
                                intake = null;
                                shoot = null;
                                transfer = null;
                                cartridge = null;
                                arm = null;
                                vision = null;
                                superCommands = null;
                                leds = null;

                                intakeCommands = null;
                                shootCommands = null;
                                transferCommands = null;
                                cartridgeCommands = null;
                                armCommands = null;
                                this.driveSimulation = new SwerveDriveSimulation(
                                                Swerve.maplesimConfig,
                                                new Pose2d(0, 0, new Rotation2d(0)));

                                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

                                swerve = new Swerve(
                                                new GyroIOSim(this.driveSimulation.getGyroSimulation()),
                                                new ModuleIOSim(this.driveSimulation.getModules()[0]),
                                                new ModuleIOSim(this.driveSimulation.getModules()[1]),
                                                new ModuleIOSim(this.driveSimulation.getModules()[2]),
                                                new ModuleIOSim(this.driveSimulation.getModules()[3]));

                                break;

                        default:
                                // Replayed robot, disable IO implementations
                                swerve = null;
                                intake = null;
                                shoot = null;
                                transfer = null;
                                cartridge = null;
                                arm = null;
                                vision = null;
                                leds = null;

                                superCommands = null;
                                intakeCommands = null;
                                shootCommands = null;
                                transferCommands = null;
                                cartridgeCommands = null;
                                armCommands = null;

                                break;
                }

                SendableChooser<Command> c = new SendableChooser<>();

                // Set up auto routines
                autoChooser = AutoBuilder.buildAutoChooser();
                // new LoggedDashboardChooser<>("Auto Choices", c); // TODO use auto builder
                // autoChooser.addDefaultOption("nothing", null);

                SmartDashboard.putData("autoChooser", autoChooser);

                // Set up SysId routines

                // Configure the button bindings
                configureButtonBindings();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * 
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                // Default command, normal field-relative drive
                leds.setDefaultCommand(LEDsCommands.setAll(leds, Color.kPurple));

                // driverController
                swerve.setDefaultCommand(
                                SwerveCommands.joystickDrive(swerve,
                                                () -> driverController.getLeftY() * 0.8,
                                                () -> driverController.getLeftX() * 0.8,
                                                () -> driverController.getRightX() * 0.6));

                driverController.LB().whileTrue(superCommands.shakeCartridge());
                driverController.leftTrigger().whileTrue(superCommands.intakeFuel(driverController.rightTrigger()));
                driverController.RB().whileTrue(superCommands.closeCartridge());
                driverController.b().whileTrue(superCommands.outtakeFuel());
                driverController.a().whileTrue(SwerveCommands.driveFaceToHub(swerve,
                                () -> driverController.getLeftY() * 0.5,
                                () -> driverController.getLeftX() * 0.5));
                driverController.PovDown().onTrue(swerve.resetGyroCommand());
                driverController.a().onTrue(superCommands.shootToHub(driverController.rightTrigger()));// הכנה של ירי
                driverController.PovUp().onTrue(shootCommands.stopBoth().alongWith(armCommands.stopArm()));// עצירת ירי

                Trigger readyToShoot = new Trigger(()->{
                        return Math.abs(swerve.getRotation().minus(SwerveCommands.angleToHub(swerve)).getDegrees() % 360) <= 5 &&
                        Math.abs(shoot.getInputs().rightGoal - shoot.getInputs().rightVelocity) < 2 && shoot.getInputs().rightVelocity > 0.5 &&
                        arm.getInputs().motorVoltage > 0.0;
                });
                readyToShoot.whileTrue(Commands.runEnd(()-> driverController.rumbleBothSides(0.5),()-> driverController.rumbleBothSides(0)));
                // operatorController

                operatorController.cross().whileTrue(shootCommands.setHoodVoltage());// ירי
                operatorController.triangle().whileTrue(shootCommands.setFeedVoltage());// הזנה

                operatorController.circle().whileTrue(cartridgeCommands.openCartridge());// פתיחת מחסנית
                operatorController.square().whileTrue(cartridgeCommands.closeCartridge());// סגירת מחסנית

                operatorController.povRight().whileTrue(armCommands.openArmManual());// פתיחת שינוי זווית
                operatorController.povLeft().whileTrue(armCommands.closeArmManual());// סגירת שינוי זווית

                operatorController.povUp().whileTrue(transferCommands.setForwoard());// טרנספר קדימה
                operatorController.povDown().whileTrue(transferCommands.setBackward());// טרנספר אחורה

                operatorController.R2().whileTrue(intakeCommands.intake());// איסוף
                operatorController.L2().whileTrue(intakeCommands.outake());// פליטה

                operatorController.R1()
                                .whileTrue(superCommands.shakeCartridge());// הכנה
                                                                                                                  // של
                                                                                                                  // ירי
                operatorController.L1().onTrue(shootCommands.stopBoth().alongWith(armCommands.stopArm()));// עצירת ירי

        }

        public void displaySimFieldToAdvantageScope() {
                if (Constants.currentMode != Constants.Mode.SIM)
                        return;

                Logger.recordOutput(
                                "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
                Logger.recordOutput(
                                "FieldSimulation/Notes", SimulatedArena.getInstance().getGamePiecesArrayByType("Note"));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
