package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shoot.Shoot;
import frc.robot.subsystems.shoot.ShootIO;
import frc.robot.subsystems.shooterArm.ShooterArm;
import frc.robot.util.BallisticCalculator.BallisticCalculator;
import frc.robot.util.BallisticCalculator.BallisticCalculatorResultWithRotation;

import java.util.List;
import java.util.function.Supplier;

public class ShootOnTheMoveCommand extends Command {
    // FIXME: switch to using the actual value
    private final double FEED_SHOOT_VOLTAGE = 0;
    private final double SHOOTER_DEFAULT_SPEED = 10.0; // [m/s]

    // shooter subsystem
    Shoot shooter;
    ShooterArm shooterArm;
    Supplier<Pose2d> shooterPosition;
    Pose3d targetPose;
    Supplier<Boolean> stopSupplier;

    public ShootOnTheMoveCommand(Supplier<Boolean> stopSupplier, Pose3d targetPose, Shoot shooter, ShooterArm shooterArm, Supplier<Pose2d> shooterPosition) {
        this.stopSupplier = stopSupplier;
        this.targetPose = targetPose;

        this.shooter = shooter;
        this.shooterArm = shooterArm;
        this.shooterPosition = shooterPosition;

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
    BallisticCalculatorResultWithRotation chooseTrajectory(List<BallisticCalculatorResultWithRotation> results, double currentVelocity) {
        BallisticCalculatorResultWithRotation best = null;
        for (var result : results) {
            if (best == null || Math.abs(currentVelocity - result.v0()) < Math.abs(currentVelocity - best.v0())) {
                best = result;
            }
        }
        return best;
    }

    // TODO: Implement
    @Override
    public void initialize() {

    }

    // TODO: Implement
    @Override
    public void execute() {
        Pose2d position;

        Translation3d translationToTarget = getTranslationToTarget(targetPose, shooterPosition.get());

        List<BallisticCalculatorResultWithRotation> results = BallisticCalculator.calculateForFuel(translationToTarget, null, BallisticCalculator.BallisticCalculatorMode.FAST); // FIXME: this isn't supposed to be null

        ShootIO.ShootIOInputs shooterInputs = new ShootIO.ShootIOInputs();
        shooter.getIO().updateInputs(shooterInputs);

        BallisticCalculatorResultWithRotation bestTrajectory = chooseTrajectory(results, (shooterInputs.leftVelocity + shooterInputs.rightVelocity) / 2);
        shooter.getIO().setHoodSetpoint(bestTrajectory.v0());

        // TODO: see if the global angle is needed or a more detailed angle
        shooterArm.getIO().setGoal(bestTrajectory.launchAngle());

        // TODO: consider the robot's movement and see if the current rotation is good
        if (shooter.getIO().atGoalHood() && shooterArm.getIO().atGoal()) {
            // do all the shooting part
            shooter.getIO().setFeedVoltage(FEED_SHOOT_VOLTAGE);
        } else {
            shooter.getIO().setFeedVoltage(0.0);
        }
    }

    // TODO: Implement
    @Override
    public void end(boolean interrupted) {
        shooter.getIO().stopFeed();
        shooter.getIO().setHoodSetpoint(SHOOTER_DEFAULT_SPEED);
    }

    // TODO: Implement
    @Override
    public boolean isFinished() {
        return stopSupplier.get();
    }
}
