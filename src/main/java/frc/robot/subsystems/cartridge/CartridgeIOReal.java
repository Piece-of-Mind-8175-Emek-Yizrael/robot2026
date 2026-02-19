package frc.robot.subsystems.cartridge;

import static frc.robot.subsystems.cartridge.CartridgeConstants.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.CartridgePose;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

public class CartridgeIOReal implements CartridgeIO {
    private final POMSparkMax motor;
    private final RelativeEncoder encoder;
    private final POMDigitalInput innerSwitch;
    private final POMDigitalInput outerSwitch;
    private final ProfiledPIDController pidController;
    private final ElevatorFeedforward ff;
    private final CartridgeTuning tuning;
    private final SparkMaxConfig config;
    private double boostKs = 0; 

    public CartridgeIOReal() {
        motor = new POMSparkMax(MOTOR_ID);
        encoder = motor.getEncoder();
        innerSwitch = new POMDigitalInput(INNER_SWITCH_CHANNEL, INNER_NORMALLY_OPEN);
        outerSwitch = new POMDigitalInput(OUTER_SWITCH_CHANNEL, OUTER_NORMALLY_OPEN);
        pidController = new ProfiledPIDController(Kp, Ki, Kd,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        pidController.setTolerance(TOLERANCE);
        ff = new ElevatorFeedforward(Ks, Kg, Kv);
        tuning = new CartridgeTuning();
        config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake)
                .smartCurrentLimit(CURRENT_LIMIT)
                .voltageCompensation(VOLTAGE_COMPENSATION)
                .inverted(INVERTED);

        config.encoder.positionConversionFactor(positionConversionFactor)
                .velocityConversionFactor(velocityConversionFactor);

        resetIfPressed();

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidController.setTolerance(TOLERANCE);
    }

    @Override
    public void updateInputs(CartridgeIOInputs inputs) {
        inputs.voltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.output = motor.getAppliedOutput();
        inputs.velocity = encoder.getVelocity();
        inputs.postion = encoder.getPosition();
        inputs.isInnerPressed = isInnerPressed();
        inputs.isOuterPressed = isOuterPressed();
        inputs.atGoal = atGoal();
        setPIDValues();
        resetIfPressed();
        Logger.recordOutput("real ks", ff.getKs());
    }

    @Override
    public void resetIfPressed() {
        if (isInnerPressed()) {
            encoder.setPosition(CLOSE_CARTRIDGE_POS);
        }

        if (isOuterPressed()) {
            encoder.setPosition(OPEN_CARTRIDGE_POS);
        }
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        motor.stop();
    }

    @Override
    public boolean isInnerPressed() {
        return innerSwitch.get();
    }

    @Override
    public boolean isOuterPressed() {
        return outerSwitch.get();
    }

    @Override
    public void goToPos(double goal) { //TODO change to const ks
            if(goal > encoder.getPosition()){
            boostKs = 0;
            if(encoder.getVelocity() < 1.9){
                ff.setKs(tuning.getKs() + 5);
            } else {
                ff.setKs(tuning.getKs());
            }
        } else {
            if(encoder.getVelocity() < 0.7){
                boostKs += 0.02;
                ff.setKs(tuning.getKs() + 3.2);
                ff.setKg(tuning.getKg() + 0.3);
            } else if(encoder.getPosition() > 0.7){
                boostKs = 0;
                ff.setKs(tuning.getKs() + 0.7);
                ff.setKg(tuning.getKg() + 0.3);
            } else if(encoder.getPosition() < 0.4){
                boostKs = 0;
                ff.setKs(tuning.getKs() - 0.2);
                ff.setKg(tuning.getKg() + 1.5);
            } else {
                boostKs = 0;
                ff.setKs(0);
                ff.setKg(tuning.getKg());
            }
        }
        motor.setVoltage(pidController.calculate(getPos(), goal) + ff.calculate(pidController.getSetpoint().velocity));
        Logger.recordOutput("feed forward value", ff.calculate(goal - encoder.getPosition()));
    }

    @Override
    public boolean atGoal() {
        return pidController.atGoal();
    }

    @Override
    public double getPos() {
        return encoder.getPosition();
    }

    @Override
    public void resetPID() {
        pidController.reset(getPos());
    }

    @Override
    public CartridgePose getCartridgePose() {
        if (isInnerPressed()) {
            return CartridgePose.CLOSE;
        }
        if (isOuterPressed()) {
            return CartridgePose.OPEN;
        } else {
            return CartridgePose.IN_MOVEMENT;
        }
    }

    @Override
    public void setPIDValues() {
        pidController.setPID(tuning.getKp(), tuning.getKi(), tuning.getKd());
        pidController
                .setConstraints(new TrapezoidProfile.Constraints(tuning.getMaxVelocity(), tuning.getMaxAcceleration()));
        ff.setKs(tuning.getKs());
        ff.setKg(tuning.getKg());
        ff.setKv(tuning.getKv());
    }

}
