package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shoot.Shoot;
import frc.robot.subsystems.shooterArm.ShooterArm;

import java.util.function.Supplier;

public class ShootOnTheMoveCommand extends Command {
    // shooter subsystem
    Shoot shoot;
    ShooterArm shooterArm;
    Supplier<Pose2d> robotPosition;
    Pose3d targetPose;

    public ShootOnTheMoveCommand(Shoot shooter, ShooterArm shooterArm, Supplier<Pose2d> shooterPosition, Pose3d targetPose) {
        this.shoot = shooter;
        this.shooterArm = shooterArm;
        this.robotPosition = robotPosition;
        this.targetPose = targetPose;

        addRequirements(shooterArm, shooter);
    }

    // TODO: Implement
    Translation3d getTranslationToTarget(Pose3d targetPose, Pose2d robotPose) {
        return new Translation3d(
            targetPose.getX() - robotPose.getX(),
            targetPose.getY() - robotPose.getY(),
            targetPose.getZ()
        );
    }



    // TODO: Implement
    @Override
    public void initialize() {

    }

    // TODO: Implement
    @Override
    public void execute() {
        Translation3d trasnlationToTarget = getTranslationToTarget(targetPose, robotPosition.get());
    }

    // TODO: Implement
    @Override
    public void end(boolean interrupted) {
    }

    // TODO: Implement
    @Override
    public boolean isFinished() {
        return false;
    }
}
