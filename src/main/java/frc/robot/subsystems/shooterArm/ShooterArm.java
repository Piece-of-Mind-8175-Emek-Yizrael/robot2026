package frc.robot.subsystems.shooterArm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooterArm.ShooterArmIO.ShooterArmIOInputs;

public class ShooterArm extends SubsystemBase{

    private final ShooterArmIO io;

    private final ShooterArmIOInputsAutoLogged inputs = new ShooterArmIOInputsAutoLogged();

    public ShooterArm(ShooterArmIO io) {
        this.io = io;

        setDefaultCommand(run(() -> io.stayInCurrentGoal()).withName("default command"));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("shooter arm", inputs);
    }

    public ShooterArmIO getIO() {
        return io;
    }
    
}
