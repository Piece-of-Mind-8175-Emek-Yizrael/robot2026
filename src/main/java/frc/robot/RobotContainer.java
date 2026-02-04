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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.POM_lib.Joysticks.PomXboxController;
import frc.robot.commands.SwerveCommands;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOReal;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // Subsystems
        Swerve swerve;

        // Controller
        private final PS5Controller driverController = new PS5Controller(0);
        private final PomXboxController operatorController = new PomXboxController(1);

        // Dashboard inputs
        private final LoggedDashboardChooser<Command> autoChooser;

        private SwerveDriveSimulation driveSimulation = null;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations
                                swerve = new Swerve(new GyroIOPigeon2(),
                                new ModuleIOReal(0),
                                new ModuleIOReal(1),
                                new ModuleIOReal(2),
                                new ModuleIOReal(3));
                                break;

                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations

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
                                break;
                }

                SendableChooser<Command> c = new SendableChooser<>();

                // Set up auto routines
                autoChooser = new LoggedDashboardChooser<>("Auto Choices", c); // TODO use auto builder

                // Set up SysId routines
                
                // Configure the button bindings
                configureButtonBindings();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                // Default command, normal field-relative drive

                swerve.setDefaultCommand(
                        SwerveCommands.joystickDrive(swerve,
                        () -> driverController.getLeftY() * -0.35,
                         () -> driverController.getLeftX() * -0.35,
                          () -> driverController.getRightX() * 0.35)
                );
                
        }

        public void displaSimFieldToAdvantageScope() {
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
                return autoChooser.get();
        }
}
