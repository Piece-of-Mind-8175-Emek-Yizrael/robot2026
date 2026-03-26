package frc.robot.subsystems.shooterArm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterArm extends SubsystemBase {

    private final ShooterArmIO io;

    private final ShooterArmIOInputsAutoLogged inputs = new ShooterArmIOInputsAutoLogged();

    public ShooterArm(ShooterArmIO io) {
        this.io = io;

        setDefaultCommand(run(() -> io.resistGravity()).withName("resist gravity"));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("shooter arm", inputs);
    }

    public ShooterArmIO getIO() {
        return io;
    }

    public ShooterArmIOInputsAutoLogged getInputs(){
        return inputs;
    }

}
