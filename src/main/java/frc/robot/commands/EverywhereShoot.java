package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.shoot.Shoot;
import frc.robot.subsystems.shooterArm.ShooterArm;

public class EverywhereShoot {

    private Swerve swerve;
    private Shoot shoot;
    private ShooterArm arm;
    private Translation2d hubPos = new Translation2d(0, 0); // Assuming the hub is at the origin

    public EverywhereShoot(Swerve swerve, Shoot shoot, ShooterArm arm) {
        this.swerve = swerve;
        this.shoot = shoot;
        this.arm = arm;
    }

    private Pose2d getPos() {
        return swerve.getPose();
    }


    private double getDistance() {
        double x = getPos().getX() - hubPos.getX();
        double y = getPos().getY() - hubPos.getY();
        return Math.sqrt(x * x + y * y);
    }

    


    
}
