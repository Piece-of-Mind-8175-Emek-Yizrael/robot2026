package frc.robot.subsystem.climb;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

import static frc.robot.subsystem.climb.ClimbConstants.*;

public class ClimbIOReal implements ClimbIO {
    private final TalonFX motor;
    private final Servo servo;
    private final POMSparkMax neo; 
    private VoltageOut voltageOut;
    private final POMDigitalInput limitSwitch;
    private final MotorOutputConfigs configs;
    private final TalonFXConfiguration config;
    private final PositionVoltage preClimbRequest;
    private final PositionVoltage ClimbRequest;
    private double currentGoal;
    private SparkMaxConfig neoConfig;

    public ClimbIOReal() {
        motor = new TalonFX(MOTOR_ID);
        servo = new Servo(SERVO_CHANNEL);
        neo = new POMSparkMax(NEO_ID);
        voltageOut = new VoltageOut(0);
        limitSwitch = new POMDigitalInput(LIMIT_SWITCH_CHANNEL, IS_NORAMLLY_OPEN);
        configs = new MotorOutputConfigs();
        configs.Inverted = InvertedValue.Clockwise_Positive;
        configs.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(configs); // should work

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

        neoConfig = new SparkMaxConfig();
        neoConfig.idleMode(IdleMode.kCoast)
                .inverted(false);

        preClimbRequest = new PositionVoltage(0).withSlot(0);
        ClimbRequest = new PositionVoltage(0).withSlot(1);

        motor.getConfigurator().apply(config);
        motor.setPosition(0);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.krakenVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.KrakenOutput = motor.getMotorOutputStatus().getValueAsDouble();
        inputs.neoVoltage = neo.getBusVoltage() * neo.getAppliedOutput();
        inputs.neoOutput = neo.getAppliedOutput();
        inputs.angle = servo.getAngle();
        inputs.isPressed = isPressed();
    }

    @Override
    public void setKrakenVoltage(double voltage) {
        motor.setVoltage(voltage);
        motor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void stopKraken() {
        motor.stopMotor();
        setKrakenVoltage(0);
    }

    @Override
    public void setNeoVoltage(double voltage) {
        neo.setVoltage(voltage);
    }

    @Override
    public void stopNeo() {
        neo.stopMotor();
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
        currentGoal = goal;
        motor.setControl(preClimbRequest.withPosition(goal));
    }

    @Override
    public void ClimbGoToPos(double goal) {
        currentGoal = goal;
        motor.setControl(ClimbRequest.withPosition(goal));
    }

    @Override
    public boolean atGoal() {
        return Math.abs(currentGoal - getPosition()) <= TOLERANCE;
    }

    @Override
    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }
}