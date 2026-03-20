package frc.robot.subsystems.cartridge;

import static frc.robot.subsystems.cartridge.CartridgeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.CartridgePose;
import frc.robot.POM_lib.Motors.POMTalonFX;
import frc.robot.POM_lib.sensors.POMDigitalInput;

public class CartridgeIOTalon implements CartridgeIO {
    private final POMTalonFX motor;
    private final POMDigitalInput innerSwitch;
    private final POMDigitalInput outerSwitch;
    private final ProfiledPIDController pidController;
    private final ElevatorFeedforward ff;
    private final CartridgeTuning tuning;
    private final TalonFXConfiguration config;

    public CartridgeIOTalon() {
        motor = new POMTalonFX(MOTOR_ID);

        innerSwitch = new POMDigitalInput(INNER_SWITCH_CHANNEL, INNER_NORMALLY_OPEN);
        outerSwitch = new POMDigitalInput(OUTER_SWITCH_CHANNEL, OUTER_NORMALLY_OPEN);

        pidController = new ProfiledPIDController(Kp, Ki, Kd,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        pidController.setTolerance(TOLERANCE);

        ff = new ElevatorFeedforward(Ks, Kg, Kv);

        tuning = new CartridgeTuning();

        config = new TalonFXConfiguration();

        config.Feedback.SensorToMechanismRatio = CONVERSION_FACTOR;
        config.TorqueCurrent.PeakForwardTorqueCurrent = CURRENT_LIMIT;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        resetIfPressed();
    }

    @Override
    public void updateInputs(CartridgeIOInputs inputs) {
        inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.output = motor.getDutyCycle().getValueAsDouble();
        inputs.velocity = motor.getVelocity().getValueAsDouble();
        inputs.postion = getPos();
        inputs.isInnerPressed = isInnerPressed();
        inputs.isOuterPressed = isOuterPressed();
        inputs.atGoal = atGoal();
        setPIDValues();
        resetIfPressed();
    }

    @Override
    public void resetIfPressed() {
        if (isInnerPressed()) {
            motor.setPosition(CLOSE_CARTRIDGE_POS);
            
        }

        if (isOuterPressed()) {
            motor.setPosition(OPEN_CARTRIDGE_POS);
        }
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setControl(new VoltageOut(voltage));
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
        setVoltage(pidController.calculate(getPos(), goal)
                + ff.calculate(pidController.getSetpoint().velocity));
    }

    @Override
    public boolean atGoal() {
        return pidController.atGoal();
    }

    @Override
    public double getPos() {
        return motor.getPosition().getValueAsDouble();
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
