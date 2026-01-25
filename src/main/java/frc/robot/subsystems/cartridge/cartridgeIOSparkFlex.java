package frc.robot.subsystems.cartridge;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;
import static frc.robot.subsystems.cartridge.cartridgeConstants.*;

public class cartridgeIOSparkFlex implements cartridgeIO {
    private final SparkFlex motor;
    private final AbsoluteEncoder encoder;
    private final POMDigitalInput limitSwitch;
    private final SparkClosedLoopController closedLoopController;
    // private final Slot0Configs driveMotorGains;
    private final SparkMaxConfig config;
    private final cartridgeTuning tuning;

    public cartridgeIOSparkFlex() {
        motor = new SparkFlex(MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();
        limitSwitch = new POMDigitalInput(LIMIT_SWITCH_CHANNEL, IS_NORMALLY_OPEN);
        closedLoopController = motor.getClosedLoopController();
        // driveMotorGains = new Slot0Configs().withKP(0).withKI(0).withKD(0).
        // withKS(0).withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        tuning = new cartridgeTuning();
        config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake)
                .smartCurrentLimit(STALL_LIMIT)
                .voltageCompensation(12);
        config.absoluteEncoder
                .positionConversionFactor(POSITION_CONVERSIN_FACTOR)
                .velocityConversionFactor(VELOCITY_CONVERSIN_FACTOR);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(kP, kI, kD, ClosedLoopSlot.kSlot0).feedForward.sv(kS, kV, ClosedLoopSlot.kSlot0);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(cartridgeIOInputs inputs) {
        inputs.voltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.output = motor.getAppliedOutput();
        inputs.velocity = encoder.getVelocity();
        inputs.postion = encoder.getPosition();
        inputs.isPressed = isPressed();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void goToPosition(double goal) {
        closedLoopController.setSetpoint(goal, ControlType.kVoltage); // need to check
    }

    @Override
    public boolean isPressed() {
        return limitSwitch.get();
    }

    @Override
    public void setPIDValues() {
        config.closedLoop.p(tuning.getKp()).i(tuning.getKi()).d(tuning.getKd()).feedForward.sv(tuning.getKs(),
                tuning.getKv());
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

}