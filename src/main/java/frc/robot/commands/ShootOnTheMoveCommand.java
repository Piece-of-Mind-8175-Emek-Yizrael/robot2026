package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.shoot.Shoot;
import frc.robot.subsystems.shoot.ShootIO;
import frc.robot.subsystems.shooterArm.ShooterArm;
import frc.robot.util.BallisticCalculator.BallisticCalculator;
import frc.robot.util.BallisticCalculator.BallisticCalculatorResult;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ShootOnTheMoveCommand extends Command {
    // FIXME: switch to using the actual value
    private final double FEED_SHOOT_VOLTAGE = 0;
    private final double SHOOTER_DEFAULT_SPEED = 10.0; // [m/s]

    // shooter subsystem
    Shoot shooter;
    ShooterArm shooterArm;
    Swerve swerve;
    
    Pose3d targetPose;
    Supplier<Boolean> stopSupplier;

    Supplier<Rotation2d> rotationSupplier = () -> new Rotation2d();
    
    Command joystickDriveAtAngleCommand;
    
    public ShootOnTheMoveCommand(Shoot shooter, ShooterArm shooterArm, Swerve swerve, Supplier<Boolean> stopSupplier, Pose3d targetPose, DoubleSupplier joystickX, DoubleSupplier joystickY) {
        this.stopSupplier = stopSupplier;
        this.targetPose = targetPose;

        this.shooter = shooter;
        this.shooterArm = shooterArm;
        this.swerve = swerve;

        joystickDriveAtAngleCommand = SwerveCommands.joystickDriveAtAngle(swerve, joystickX, joystickY, rotationSupplier);
    }

    Translation3d getTranslationToTarget(Pose3d targetPose, Pose2d robotPose) {
        return new Translation3d(
            targetPose.getX() - robotPose.getX(),
            targetPose.getY() - robotPose.getY(),
            targetPose.getZ()
        );
        
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(joystickDriveAtAngleCommand);
        addRequirements(shooterArm, shooter);
    }

    // TODO: Implement
    @Override
    public void execute() {
        Pose2d position = swerve.getPose();

        Translation3d translationToTarget = getTranslationToTarget(targetPose, swerve.getPose());

        ChassisSpeeds velocitySpeeds = swerve.getChassisSpeeds();
        Translation2d velocityTranslation = new Translation2d(velocitySpeeds.vxMetersPerSecond, velocitySpeeds.vyMetersPerSecond);

        // TODO: Check wheather this is correct
        Transform2d posOffset = position.minus(targetPose.toPose2d());
        double angleRadians = Math.atan2(posOffset.getX(), posOffset.getY());
        velocityTranslation = velocityTranslation.rotateBy(Rotation2d.fromRadians(angleRadians));


        ShootIO.ShootIOInputs shooterInputs = new ShootIO.ShootIOInputs();
        shooter.getIO().updateInputs(shooterInputs);

        BallisticCalculator.getInstance().updateParameters(translationToTarget, velocityTranslation);
        BallisticCalculatorResult trajectory = BallisticCalculator.getInstance().getLatestResults();

        shooter.getIO().setHoodSetpoint(trajectory.v0() * 2); // TODO: check the said setPoint is the momentary velocity at the edge of the flywheel

        rotationSupplier = () -> swerve.getRotation().plus(Rotation2d.fromDegrees(trajectory.dRobotAngle()));
        
        // TODO: see if the global angle is needed or a more detailed angle
        shooterArm.getIO().setGoal(trajectory.launchAngle());

        // TODO: consider the robot's movement and see if the current rotation is good
        if (shooter.getIO().atGoalHood() && shooterArm.getIO().atGoal()) {
            // do all the shooting part
            shooter.getIO().setFeedVoltage(FEED_SHOOT_VOLTAGE);
        } else {
            shooter.getIO().stopFeed();
        }
    }

    // TODO: Implement
    @Override
    public void end(boolean interrupted) {
        shooter.getIO().stopFeed();
        shooter.getIO().setHoodSetpoint(SHOOTER_DEFAULT_SPEED);
        joystickDriveAtAngleCommand.end(true); // TODO: is this supposed to be "true"?
    }

    // TODO: Implement
    @Override
    public boolean isFinished() {
        return stopSupplier.get();
    }
}
