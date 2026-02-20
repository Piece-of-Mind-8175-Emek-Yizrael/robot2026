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

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.Motors.POMTalonFX;

public class ShootIOReal implements ShootIO {

    private final POMTalonFX leftHoodMotor;
    private final POMTalonFX rightHoodMotor;

    private final POMSparkMax feedMotor;
    private final RelativeEncoder encoder;

    private final SparkMaxConfig feedConfig;
    private final TalonFXConfiguration hoodRightConfig;
    private final TalonFXConfiguration hoodLeftConfig;

    private double rightGoalHoodVelocity = 0.0;
    private double leftGoalHoodVelocity = 0.0;
    private double feedGoalVelocity = 0.0;

    private ProfiledPIDController feedController;

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);

    public ShootIOReal() {
        leftHoodMotor = new POMTalonFX(LEFT_HOOD_MOTOR_ID);
        rightHoodMotor = new POMTalonFX(RIGHT_HOOD_MOTOR_ID);
        feedMotor = new POMSparkMax(FEED_MOTOR_ID);
        encoder = feedMotor.getEncoder();
        
        //right hood
        hoodRightConfig = new TalonFXConfiguration();
        Slot0Configs rightHoodSlot0 = new Slot0Configs()
        .withKV(kvRightHood).withKS(ksRightHood).withKP(kpRightHood).withKI(kiRightHood).withKD(kdRightHood);
        
        hoodRightConfig.Slot0 = rightHoodSlot0;
        hoodRightConfig.Feedback.SensorToMechanismRatio = hoodGearRatio;
        hoodRightConfig.TorqueCurrent.PeakForwardTorqueCurrent = slipCurrent;
        hoodRightConfig.TorqueCurrent.PeakReverseTorqueCurrent = -slipCurrent;
        hoodRightConfig.CurrentLimits.StatorCurrentLimit = slipCurrent;
        hoodRightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodRightConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampRate;
        hoodRightConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = rampRate;
        hoodRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        hoodRightConfig.MotorOutput.withInverted(RIGHT_HOOD_DIRECTION);
        
        
        //left hood
        hoodLeftConfig = new TalonFXConfiguration();
        Slot0Configs leftHoodSlot0 = new Slot0Configs()
        .withKV(kvLeftHood).withKS(ksLeftHood).withKP(kpLeftHood).withKI(kiLeftHood).withKD(kdLefthHood);
        
        hoodLeftConfig.Slot0 = leftHoodSlot0;
        hoodLeftConfig.Feedback.SensorToMechanismRatio = hoodGearRatio;
        hoodLeftConfig.TorqueCurrent.PeakForwardTorqueCurrent = slipCurrent;
        hoodLeftConfig.TorqueCurrent.PeakReverseTorqueCurrent = -slipCurrent;
        hoodLeftConfig.CurrentLimits.StatorCurrentLimit = slipCurrent;
        hoodLeftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodLeftConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampRate;
        hoodLeftConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = rampRate;
        hoodLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        hoodLeftConfig.MotorOutput.withInverted(LEFT_HOOD_DIRECTION);
        
        tryUntilOk(5, () -> leftHoodMotor.getConfigurator().apply(hoodLeftConfig, 0.25));
        tryUntilOk(5, () -> rightHoodMotor.getConfigurator().apply(hoodRightConfig, 0.25));
        
        // leftHoodMotor.setControl(new Follower(rightHoodMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        
        //feed config
        feedConfig = new SparkMaxConfig();
        feedController = new ProfiledPIDController(kpFeed, kiFeed, kdFeed,
        new TrapezoidProfile.Constraints(maxVelocityFeed, maxAccelerationFeed));
        feedController.setTolerance(feedTolerance);
        
        feedConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(feedCurrentLimit)
                .voltageCompensation(12.0)
                .openLoopRampRate(rampRate)
                .closedLoopRampRate(rampRate)
                .inverted(true);

        
        feedConfig.encoder
                // .inverted(feedInverted)
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
        inputs.bothAtGoal = atGoalHood();
        
        //left hood motor
        inputs.leftHoodConnected = leftHoodMotor.isConnected();
        inputs.leftVoltage = leftHoodMotor.getMotorVoltage().getValueAsDouble();
        inputs.leftVelocity = leftHoodMotor.getVelocity().getValueAsDouble();
        inputs.leftAppliedVoltage = leftHoodMotor.getMotorVoltage().getValueAsDouble() * leftHoodMotor.getDutyCycle().getValueAsDouble();
        inputs.leftAtGoal = Math.abs(leftHoodMotor.getVelocity().getValueAsDouble() - leftGoalHoodVelocity) <= hoodTolerance;
        inputs.leftGoal = leftGoalHoodVelocity;
        
        //right hood motor
        inputs.rightHoodConnected = rightHoodMotor.isConnected();
        inputs.rightVoltage = rightHoodMotor.getMotorVoltage().getValueAsDouble();
        inputs.rightVelocity = rightHoodMotor.getVelocity().getValueAsDouble();
        inputs.rightAppliedVoltage = rightHoodMotor.getMotorVoltage().getValueAsDouble() * rightHoodMotor.getDutyCycle().getValueAsDouble();
        inputs.rightAtGoal = Math.abs(rightHoodMotor.getVelocity().getValueAsDouble() - rightGoalHoodVelocity) <= hoodTolerance;
        inputs.rightGoal = rightGoalHoodVelocity;

        //transfer motor
        inputs.feedConnected = feedMotor.getFirmwareVersion() != 0;
        inputs.feedAppliedVoltage = feedMotor.getAppliedOutput();
        inputs.feedVelocity = encoder.getVelocity();
        inputs.feedVoltage = feedMotor.getAppliedOutput() * feedMotor.getBusVoltage();
        inputs.feedAtGoal = feedController.atGoal();
        inputs.feedGoal = feedGoalVelocity;

    }

    @Override
    public void setHoodVoltage(double voltage) {
        leftGoalHoodVelocity = 0.0;
        rightGoalHoodVelocity = 0.0;
        rightHoodMotor.setControl(new VoltageOut(voltage));
        leftHoodMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setFeedVoltage(double voltage) {
        feedGoalVelocity = 0.0;
        feedMotor.setVoltage(voltage);
    }

    @Override
    public void stopHood() {
        leftGoalHoodVelocity = 0.0;
        rightGoalHoodVelocity = 0.0;
        setHoodVoltage(0);
    }

    @Override
    public void stopFeed() {
        feedGoalVelocity = 0.0;
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
        leftGoalHoodVelocity = targetVelocity;
        rightGoalHoodVelocity = targetVelocity;
        rightHoodMotor.setControl(velocityVoltage.withVelocity(targetVelocity));
        leftHoodMotor.setControl(velocityVoltage.withVelocity(targetVelocity));
    }

    @Override
    public boolean atGoalHood() {//TODO check for both
        double leftCurrentVelocity = leftHoodMotor.getVelocity().getValueAsDouble();
        double rightCurrentVelocity = rightHoodMotor.getVelocity().getValueAsDouble();
        boolean leftAtGoal = false;
        boolean rightAtGoal = false;
        if(Math.abs(leftCurrentVelocity - leftGoalHoodVelocity) <= hoodTolerance && !leftAtGoal){
            leftAtGoal = true;
        }
        if(Math.abs(rightCurrentVelocity - rightGoalHoodVelocity) <= hoodTolerance && !rightAtGoal){
            rightAtGoal = true;
        }
        return rightAtGoal /*&& leftAtGoal*/;
    }

    @Override
    public void setFeedSetpoint(double goal) {
        feedGoalVelocity = goal;
        feedController.setGoal(goal);
        feedMotor.setVoltage(feedController.calculate(encoder.getVelocity()));
    }


    @Override
    public boolean atGoalFeed() {
        return feedController.atGoal();
    }
}