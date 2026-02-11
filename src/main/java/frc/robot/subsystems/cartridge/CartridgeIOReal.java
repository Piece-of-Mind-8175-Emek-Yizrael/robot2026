package frc.robot.subsystems.cartridge;

import static frc.robot.subsystems.cartridge.CartridgeConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

public class CartridgeIOReal implements CartridgeIO {
    private final POMSparkMax motor;
    private final RelativeEncoder encoder;
    private final POMDigitalInput innerSwitch;
    private final POMDigitalInput outerSwitch;
    private final ProfiledPIDController pidController;
    private final CartridgeTuning tuning;
    private final SparkMaxConfig config;

    public CartridgeIOReal() {
        motor = new POMSparkMax(MOTOR_ID);
        encoder = motor.getEncoder();
        innerSwitch = new POMDigitalInput(INNER_SWITCH_CHANNEL, INNER_NORMALLY_OPEN);
        outerSwitch = new POMDigitalInput(OUTER_SWITCH_CHANNEL, OUTER_NORMALLY_OPEN);
        pidController = new ProfiledPIDController(Kp, Ki, Kd,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        tuning = new CartridgeTuning();
        config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake)
                .smartCurrentLimit(STALL_LIMIT)
                .voltageCompensation(VOLTAGE_COMPENSATION)
                .inverted(INVERTED);

        config.encoder.positionConversionFactor(1.0)
                .velocityConversionFactor(1.0 / 60.0);

        resetIfPressed();

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidController.setTolerance(TOLERANCE);
    }

    @Override
    public void updateInputs(CartridgeIOInputs inputs) {
        inputs.voltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.output = motor.getAppliedOutput();
        inputs.postion = encoder.getPosition();
        inputs.isInnerPressed = isInnerPressed();
        inputs.isOuterPressed = isOuterPressed();
        inputs.atGoal = atGoal();
        resetIfPressed();
    }

    @Override
    public void resetIfPressed() {
        if(isInnerPressed()) {
            encoder.setPosition(CLOSE_CARTRIDGE_POS);
        }
        
        if(isOuterPressed()) {
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
    public void goToPos(double goal) {
        motor.setVoltage(pidController.calculate(getPos(), goal));
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
    public void setPIDValues() {
        pidController.setPID(tuning.getKp(), tuning.getKi(), tuning.getKd());
        pidController
                .setConstraints(new TrapezoidProfile.Constraints(tuning.getMaxVelocity(), tuning.getMaxAcceleration()));
    }

}
