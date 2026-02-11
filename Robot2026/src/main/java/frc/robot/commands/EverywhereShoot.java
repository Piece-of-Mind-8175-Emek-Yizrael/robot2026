package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.shoot.Shoot;
import frc.robot.subsystems.shooterArm.ShooterArm;

public class EverywhereShoot {

    private Swerve swerve;
    private Shoot shoot;
    private ShooterArm arm;
    private Translation2d hubPos = new Translation2d(0, 0); // Assuming the hub is at the origin
    private final double L1Distance =  0.0;
    private final double L2Distance =  0.0;
    private final double L3Distance =  0.0;
    private final double L4Distance =  0.0;
    private final double L5Distance =  0.0;
    private final double L6Distance =  0.0;
    private final double deliveryDistance =  0.0;


    private final double L1Angle = 0.0;
    private final double L2Angle = 0.0;
    private final double L3Angle = 0.0;
    private final double L4Angle = 0.0;
    private final double L5Angle = 0.0;
    private final double L6Angle = 0.0;
    private final double deliveryAngle = 0.0;

    public EverywhereShoot(Swerve swerve, Shoot shoot, ShooterArm arm) {
        this.swerve = swerve;
        this.shoot = shoot;
        this.arm = arm;
    }

    private Pose2d getPos() {
        return swerve.getPose();
    }


    private double getDistance() {
        return swerve.getDistanceFromHub();
    }

    public double getArmAngleByPos() {
        double distance = getDistance();
        if(getPos().getX() < deliveryDistance){
            return deliveryAngle;
        } else if (distance < L1Distance) {
            return L1Angle; 
        } else if (distance < L2Distance) {
            return L2Angle;
        } else if (distance < L3Distance) {
            return L3Angle;
        } else if (distance < L4Distance) {
            return L4Angle;
        } else if (distance < L5Distance) {
            return L5Angle;
        } else if (distance < L6Distance) {
            return L6Angle;
        } else {
            return 0.0; // Default angle for out of range
        }
    }

    public double getArmAngleByInterpolation(double distance) {
        InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
        double targetAngle = 0.0;

        // up to 3M, enter angle by radios
        //0 - 1M, dont have to check 0.3,6,9
        angleMap.put(0.0, 0.0);
        angleMap.put(0.3, 0.0);
        angleMap.put(0.5, 0.0);
        angleMap.put(0.6, 0.0);
        angleMap.put(0.9, 0.0);
        angleMap.put(1.0, 0.0);

        //1 - 2M, dont have to check 1.3,6,9
        angleMap.put(1.3, 0.0);
        angleMap.put(1.5, 0.0);
        angleMap.put(1.6, 0.0);
        angleMap.put(1.9, 0.0);
        angleMap.put(2.0, 0.0);

        //2 - 3M, dont have to check 2.3,6,9
        angleMap.put(2.3, 0.0);
        angleMap.put(2.5, 0.0);
        angleMap.put(2.6, 0.0);
        angleMap.put(2.9, 0.0);
        angleMap.put(3.0, 0.0);    
        
        targetAngle = angleMap.get(distance);

        return targetAngle; 
    }

    public Command setArmByDistance(){
        return new Command() {
            double currentAngle = getArmAngleByPos();

            {
                addRequirements(swerve, arm);
            }

            @Override
            public void initialize() {
                arm.getIO().resetPID(getArmAngleByPos());
                currentAngle = getArmAngleByPos();
            }

            @Override
            public void execute() {
                if(currentAngle != getArmAngleByPos()) {
                    currentAngle = getArmAngleByPos();
                    arm.getIO().resetPID(currentAngle);
                } else {
                    arm.getIO().setGoal(getArmAngleByPos());
                }
                
            }

            @Override
            public void end(boolean interrupted) {
                arm.getIO().stopMotor();
            }

            @Override
            public boolean isFinished() {
                return false; 
                
            }
        };
    }

    
}
