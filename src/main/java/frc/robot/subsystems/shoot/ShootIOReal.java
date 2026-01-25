package frc.robot.subsystems.shoot;

import static frc.robot.subsystems.shoot.ShootConstants.LEFT_HOOD_MOTOR_ID;
import static frc.robot.subsystems.shoot.ShootConstants.RIGHT_HOOD_MOTOR_ID;
import static frc.robot.subsystems.shoot.ShootConstants.TRANSFER_MOTOR_ID;
import static frc.robot.subsystems.shoot.ShootConstants.encoderPositionFactor;
import static frc.robot.subsystems.shoot.ShootConstants.rampRate;
import static frc.robot.subsystems.shoot.ShootConstants.slipCurrent;
import static frc.robot.subsystems.shoot.ShootConstants.transferCurrentLimit;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import static frc.robot.util.SparkUtil.tryUntilOk;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.Motors.POMTalonFX;

public class ShootIOReal implements ShootIO {

    private final POMTalonFX leftHoodMotor;
    private final POMTalonFX rightHoodMotor;

    private final POMSparkMax transferMotor;
    private final RelativeEncoder encoder;

    private final SparkMaxConfig transferConfig;
    private final TalonFXConfiguration hoodConfig;

    public ShootIOReal() {
        leftHoodMotor = new POMTalonFX(LEFT_HOOD_MOTOR_ID);
        rightHoodMotor = new POMTalonFX(RIGHT_HOOD_MOTOR_ID);
        transferMotor = new POMSparkMax(TRANSFER_MOTOR_ID);
        encoder = transferMotor.getEncoder();

        hoodConfig = new TalonFXConfiguration();
        transferConfig = new SparkMaxConfig();

        hoodConfig.Feedback.SensorToMechanismRatio = encoderPositionFactor;
        hoodConfig.TorqueCurrent.PeakForwardTorqueCurrent = slipCurrent;
        hoodConfig.TorqueCurrent.PeakReverseTorqueCurrent = -slipCurrent;
        hoodConfig.CurrentLimits.StatorCurrentLimit = slipCurrent;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampRate;
        hoodConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = rampRate;
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        tryUntilOk(5, () -> leftHoodMotor.getConfigurator().apply(hoodConfig, 0.25));
        tryUntilOk(5, () -> rightHoodMotor.getConfigurator().apply(hoodConfig, 0.25));
        
        leftHoodMotor.setControl(new Follower(rightHoodMotor.getDeviceID(), MotorAlignmentValue.Aligned));


        transferConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(transferCurrentLimit)
                .voltageCompensation(12.0)
                .openLoopRampRate(rampRate)
                .closedLoopRampRate(rampRate)
                .inverted(false);

        
        transferConfig.encoder
                .inverted(false)
                .positionConversionFactor(1.0)
                .velocityConversionFactor(1.0)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

        
        tryUntilOk(
                transferMotor,
                5,
                () -> transferMotor.configure(
                        transferConfig, ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters));



    }

    @Override
    public void updateInputs(ShootIOInputs inputs) {
        //left hood motor
        inputs.leftHoodConnected = leftHoodMotor.isConnected();
        inputs.leftVoltage = leftHoodMotor.getMotorVoltage().getValueAsDouble();
        inputs.leftVelocity = leftHoodMotor.getVelocity().getValueAsDouble();
        inputs.leftAppliedVoltage = leftHoodMotor.getMotorVoltage().getValueAsDouble();

        //right hood motor
        inputs.rightHoodConnected = rightHoodMotor.isConnected();
        inputs.rightVoltage = rightHoodMotor.getMotorVoltage().getValueAsDouble();
        inputs.rightVelocity = rightHoodMotor.getVelocity().getValueAsDouble();
        inputs.rightAppliedVoltage = rightHoodMotor.getMotorVoltage().getValueAsDouble();

        //transfer motor
        inputs.transferConnected = transferMotor.getFirmwareVersion() != 0;
        inputs.transferVoltage = transferMotor.getBusVoltage();
        inputs.transferVelocity = encoder.getVelocity();
        inputs.transferAppliedVoltage = transferMotor.getAppliedOutput() * transferMotor.getBusVoltage();

    }

    @Override
    public void setHoodVoltage(double voltage) {
        rightHoodMotor.setVoltage(voltage);
    }

    @Override
    public void setTransferVoltage(double voltage) {
        transferMotor.setVoltage(voltage);
    }

    @Override
    public void stopHood() {
        rightHoodMotor.setVoltage(0.0);
    }

    @Override
    public void stopTransfer() {
        transferMotor.setVoltage(0.0);
    }

    @Override
    public void setVoltage(double hoodVoltage, double transferVoltage) {
        setHoodVoltage(hoodVoltage);
        setTransferVoltage(transferVoltage);
    }

    @Override
    public void stopBoth() {
        stopHood();
        stopTransfer();
    }
}