package frc.robot.subsystem.climb;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.POM_lib.sensors.POMDigitalInput;

import static frc.robot.subsystem.climb.ClimbConstants.*;

public class ClimbIOReal implements ClimbIO {
    private final TalonFX motor;
    private final Servo servo;
    private VoltageOut voltageOut;
    private final POMDigitalInput limitSwitch;
    private final TalonFXConfiguration config;
    private final PositionVoltage preClimbRequest;
    private final PositionVoltage ClimbRequest;
    private double goal;

    public ClimbIOReal() {
        motor = new TalonFX(MOTOR_ID);
        servo = new Servo(SERVO_CHANNEL);
        voltageOut = new VoltageOut(0);
        limitSwitch = new POMDigitalInput(LIMIT_SWITCH_CHANNEL, IS_NORAMLLY_OPEN);

        config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // need to check
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Feedback.RotorToSensorRatio = GEAR_RATIO;
        config.TorqueCurrent.PeakForwardTorqueCurrent = CURRENT_LIMIT;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        Slot0Configs preClimbSlot = new Slot0Configs().withKP(Kp).withKI(Ki).withKD(Kd).withKS(Ks)
                .withKG(BEFORE_CLIMB_Kg).withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        Slot1Configs ClimbSlot = new Slot1Configs().withKP(Kp).withKI(Ki).withKD(Kd).withKS(Ks).withKG(CLIMB_Kg)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        config.Slot0 = preClimbSlot;
        config.Slot1 = ClimbSlot;

        preClimbRequest = new PositionVoltage(0).withSlot(0);
        ClimbRequest = new PositionVoltage(0).withSlot(1);

        motor.getConfigurator().apply(config);
        motor.setPosition(0);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.output = motor.getMotorOutputStatus().getValueAsDouble();
        inputs.angle = servo.getAngle();
        inputs.isPressed = isPressed();
    }

    @Override
    public void setMotorVoltage(double voltage) {
        motor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void stopMotor() {
        setMotorVoltage(0);
    }

    @Override
    public void setServoAngle(double angle) {
        servo.setAngle(angle);
    }

    @Override
    public boolean isPressed() {
        return limitSwitch.get();
    }

    @Override
    public void preClimbGoToPos(double goal) {
        motor.setControl(preClimbRequest.withPosition(goal));
    }

    @Override
    public void ClimbGoToPos(double goal) {
        motor.setControl(ClimbRequest.withPosition(goal));
    }

    @Override
    public boolean atGoal() {
        return Math.abs(getPosition() - goal) <= TOLERANCE;
    }

    @Override
    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }
}
