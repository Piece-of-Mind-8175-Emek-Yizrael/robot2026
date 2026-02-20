package frc.robot.subsystems.shoot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoot extends SubsystemBase{

    private final ShootIO io;
    private final ShootIOInputsAutoLogged inputs = new ShootIOInputsAutoLogged();

    public Shoot(ShootIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("shoot", inputs);
    }

    public ShootIO getIO(){
        return io;
    }
    
}
