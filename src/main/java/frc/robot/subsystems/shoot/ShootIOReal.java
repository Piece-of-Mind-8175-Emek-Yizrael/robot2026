package frc.robot.subsystems.shoot;

import static frc.robot.subsystems.shoot.ShootConstants.*;

import static frc.robot.util.PhoenixUtil.tryUntilOk;


import static frc.robot.util.SparkUtil.tryUntilOk;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.Motors.POMTalonFX;

public class ShootIOReal implements ShootIO {

    private final POMTalonFX leftHoodMotor;
    private final POMTalonFX rightHoodMotor;

    private final POMSparkMax feedMotor;
    private final RelativeEncoder encoder;

    private final SparkMaxConfig feedConfig;
    private final TalonFXConfiguration hoodConfig;

    private double goalHoodVelocity = 0.0;

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);

    public ShootIOReal() {
        leftHoodMotor = new POMTalonFX(LEFT_HOOD_MOTOR_ID);
        rightHoodMotor = new POMTalonFX(RIGHT_HOOD_MOTOR_ID);
        feedMotor = new POMSparkMax(FEED_MOTOR_ID);
        encoder = feedMotor.getEncoder();

        hoodConfig = new TalonFXConfiguration();
        feedConfig = new SparkMaxConfig();

        Slot0Configs hoodSlot0 = new Slot0Configs()
        .withKV(kv).withKS(ks).withKP(kp).withKI(ki).withKD(kd);
    

        hoodConfig.Slot0 = hoodSlot0;

        hoodConfig.Feedback.SensorToMechanismRatio = hoodGearRatio;
        hoodConfig.TorqueCurrent.PeakForwardTorqueCurrent = slipCurrent;
        hoodConfig.TorqueCurrent.PeakReverseTorqueCurrent = -slipCurrent;
        hoodConfig.CurrentLimits.StatorCurrentLimit = slipCurrent;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampRate;
        hoodConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = rampRate;
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        hoodConfig.MotorOutput.withInverted(HOOD_DIRECTION);

        tryUntilOk(5, () -> leftHoodMotor.getConfigurator().apply(hoodConfig, 0.25));
        tryUntilOk(5, () -> rightHoodMotor.getConfigurator().apply(hoodConfig, 0.25));
        
        leftHoodMotor.setControl(new Follower(rightHoodMotor.getDeviceID(), MotorAlignmentValue.Opposed));


        feedConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(feedCurrentLimit)
                .voltageCompensation(12.0)
                .openLoopRampRate(rampRate)
                .closedLoopRampRate(rampRate)
                .inverted(feedInverted);

        
        feedConfig.encoder
                .inverted(feedInverted)
                .positionConversionFactor(feedGearRatio)
                .velocityConversionFactor(feedEncoderVelocityFactor)
                .uvwMeasurementPeriod(20);

        
        tryUntilOk(
                feedMotor,
                5,
                () -> feedMotor.configure(
                        feedConfig, ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters));



    }

    @Override
    public void updateInputs(ShootIOInputs inputs) {
        
        inputs.goalHoodVelocity = goalHoodVelocity;
        
        //left hood motor
        inputs.leftHoodConnected = leftHoodMotor.isConnected();
        inputs.leftVoltage = leftHoodMotor.getMotorVoltage().getValueAsDouble();
        inputs.leftVelocity = leftHoodMotor.getVelocity().getValueAsDouble();
        inputs.leftAppliedVoltage = leftHoodMotor.getMotorVoltage().getValueAsDouble() * leftHoodMotor.getDutyCycle().getValueAsDouble();

        //right hood motor
        inputs.rightHoodConnected = rightHoodMotor.isConnected();
        inputs.rightVoltage = rightHoodMotor.getMotorVoltage().getValueAsDouble();
        inputs.rightVelocity = rightHoodMotor.getVelocity().getValueAsDouble();
        inputs.rightAppliedVoltage = rightHoodMotor.getMotorVoltage().getValueAsDouble() * rightHoodMotor.getDutyCycle().getValueAsDouble();

        //transfer motor
        inputs.feedConnected = feedMotor.getFirmwareVersion() != 0;
        inputs.feedVoltage = feedMotor.getBusVoltage();
        inputs.feedVelocity = encoder.getVelocity();
        inputs.feedAppliedVoltage = feedMotor.getAppliedOutput() * feedMotor.getBusVoltage();

    }

    @Override
    public void setHoodVoltage(double voltage) {
        goalHoodVelocity = 0.0;
        rightHoodMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setFeedVoltage(double voltage) {
        feedMotor.setVoltage(voltage);
    }

    @Override
    public void stopHood() {
        goalHoodVelocity = 0.0;
        setHoodVoltage(0);
    }

    @Override
    public void stopFeed() {
        setFeedVoltage(0);
    }

    @Override
    public void setBoth(double hoodVoltage, double feedVoltage) {
        setHoodVoltage(hoodVoltage);
        setFeedVoltage(feedVoltage);
    }

    @Override
    public void stopBoth() {
        stopHood();
        stopFeed();
    }

    @Override
    public void setHoodSetpoint(double targetVelocity) {
        goalHoodVelocity = targetVelocity;
        rightHoodMotor.setControl(velocityVoltage.withVelocity(targetVelocity));
    }

    @Override
    public boolean atGoal() {
        double currentVelocity = rightHoodMotor.getVelocity().getValueAsDouble();
        return Math.abs(currentVelocity - goalHoodVelocity) <= hoodTolerance;
    }
}