package frc.robot.commands;

import static frc.robot.subsystem.climb2.ClimbConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.climb2.Climb;

public class ClinbCommands2 {

    private Climb climb;
    
    public ClinbCommands2(Climb climb) {
        this.climb = climb;
    }

    public Command setClimbMotor(){
        return Commands.runEnd(() -> climb.getIo().setKrakenVoltage(0.5), climb.getIo()::stopKraken, climb);
    }
    
    public Command setAngleMotor(){
        return Commands.runEnd(() -> climb.getIo().setNeoVoltage(0.5), climb.getIo()::stopNeo, climb);
    }

    public Command setServo(double angle){
        return Commands.runOnce(() -> climb.getIo().setServoAngle(angle), climb);
    }

    public Command openServo(){
        return setServo(OPEN_SERVO_POS);
    }

    public Command closeServo(){
        return setServo(CLOSE_SERVO_POS);
    }

    public Command armGoToAngle(double goal){
        return Commands.runEnd(() -> climb.getIo().setNeoGoal(goal), climb.getIo()::stopNeo, climb)
                        .until(() -> climb.getIo().atNeoGoal());
    }

    public Command closeArm(){
        return armGoToAngle(CLOSE_ARM_ANGLE).withTimeout(2);
    }

    public Command openArm(){
        return armGoToAngle(OPEN_ARM_ANGLE).withTimeout(2);
    }

    public Command preClimbGoToPos(double goal){
        return Commands.runEnd(() -> climb.getIo().ClimbGoToPos(goal), climb.getIo()::stopKraken, climb)
                        .until(climb.getIo()::atKrakenGoal);
    }

    public Command climbGoToPos(double goal){
        return Commands.runEnd(() -> climb.getIo().ClimbGoToPos(goal), climb.getIo()::stopKraken, climb)
                        .until(() -> (climb.getIo().atKrakenGoal() || climb.getIo().isPressed()));
    }

    public Command openClimb(){
        return Commands.sequence(
            openServo()
            ,openArm()
            // ,WaitCommand(0.2)
            ,climbGoToPos(OPEN_CLIME_GOAL)
            ,closeServo()

        );
    }

    public Command closeClimb(){
        return Commands.sequence(
            climbGoToPos(CLOSE_CLIME_GOAL)
            ,openServo()
            ,preClimbGoToPos(CLOSE_CLIME_GOAL)
            ,closeArm()
            ,closeServo()
        );
    }
}
