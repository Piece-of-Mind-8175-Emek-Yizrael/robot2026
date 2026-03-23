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

public class CartridgeIOSpark implements CartridgeIO {
    private final POMSparkMax motor;
    private final RelativeEncoder encoder;
    private final POMDigitalInput innerSwitch;
    private final POMDigitalInput outerSwitch;
    private final ProfiledPIDController pidController;
    private final ElevatorFeedforward ff;
    private final CartridgeTuning tuning;
    private final SparkMaxConfig config;

    public CartridgeIOSpark() {
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

        config.encoder.positionConversionFactor(CONVERSION_FACTOR)
                .velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.follow(motor, true);

        resetIfPressed();
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
            encoder.setPosition(openCartridgePos);
        }

        if (isOuterPressed()) {
            encoder.setPosition(closeCartridgePos);
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
    public void goToPos(double goal) {
        if (goal > encoder.getPosition()) {
            if (encoder.getPosition() < 0.1 || motor.getAppliedOutput() < 0.2) {
                ff.setKs(Ks + 2);
                ff.setKg(Kg + 2);
                pidController.setP(Kp + 0.3);
            } else if (encoder.getPosition() < 0.4) {
                ff.setKs(Ks + 1.5);
                ff.setKg(Kg);
                pidController.setP(Kp);
            } else if (encoder.getPosition() < 0.7) {
                ff.setKs(Ks + 1);
                ff.setKg(Kg);
                pidController.setP(Kp + 0.7);
            } else if (encoder.getPosition() > 0.7 && !isInnerPressed()) {
                ff.setKs(Ks);
                ff.setKg(Kg);
                pidController.setP(Kp);
            } else {
                ff.setKs(Ks);
                ff.setKg(Kg);
                pidController.setP(Kp);
            }
        } else {
            if (encoder.getPosition() > 0.9) {
                ff.setKs(Ks + 7);
                ff.setKg(Kg - 1.2);
                pidController.setP(Kp + 0.3);
            } else if (encoder.getPosition() > 0.7) {
                ff.setKs(Ks + 3.5);
                ff.setKg(Kg);
                pidController.setP(Kp);
            } else if (encoder.getPosition() > 0.4) {
                ff.setKs(Ks + 2.5);
                ff.setKg(Kg);
                pidController.setP(Kp);
            } else if (encoder.getPosition() < 0.4 && !isOuterPressed()) {
                ff.setKs(Ks + 1.8);
                ff.setKg(Kg);
                pidController.setP(Kp + 0.2);
            } else {
                ff.setKs(Ks);
                ff.setKg(Kg);
                pidController.setP(Kp);
            }
        }
        motor.setVoltage(
                pidController.calculate(getPos(), goal) + ff.calculate(pidController.getSetpoint().velocity));
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
