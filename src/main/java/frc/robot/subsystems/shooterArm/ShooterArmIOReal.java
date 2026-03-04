package frc.robot.subsystems.shooterArm;

import static frc.robot.subsystems.shooterArm.ShooterArmConstants.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

public class ShooterArmIOReal implements ShooterArmIO {

    private POMSparkMax motor;
    private RelativeEncoder encoder;
    private ProfiledPIDController pidController;
    private final ArmFeedforward feedforward;
    private double currentGoal = 0.0;
    private boolean manualMode = false;
    private SparkMaxConfig config;
    private POMDigitalInput sensor;

    public ShooterArmIOReal() {
        motor = new POMSparkMax(MOTOR_ID);
        encoder = motor.getEncoder();
        sensor = new POMDigitalInput(SENSOR_ID);

        pidController = new ProfiledPIDController(kp, ki, kd,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        pidController.setTolerance(TOLERANCE);
        feedforward = new ArmFeedforward(ks, kg, kv);

        config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake)
                .smartCurrentLimit(currentLimit)
                .voltageCompensation(voltageCompensation)
                .smartCurrentLimit(currentLimit);

        config.encoder.positionConversionFactor(gearRatio)
                .velocityConversionFactor(velocityConversionFactor)
                .uvwMeasurementPeriod(20);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        resetIfPrees();
    }

    @Override
    public void updateInputs(ShooterArmIOInputs inputs) {
        inputs.motorConnected = motor.getFirmwareVersion() != 0;
        inputs.armPosition = encoder.getPosition();
        inputs.armVelocity = encoder.getVelocity();
        inputs.motorVoltage = motor.getAppliedOutput();
        inputs.motorAppliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.atGoal = atGoal();
        inputs.sensor = sensor.get();
        resetIfPrees();
    }

    @Override
    public void resetIfPrees() {
        if(sensor.get()){
            zeroPosition();
            feedforward.setKg(0.5);
        } else if (encoder.getPosition() < 0.7){
            feedforward.setKg(kg);
        } else {
            feedforward.setKg(kg + 0.4);
        }

    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void stopMotor() {
        motor.stop();
    }

    @Override
    public void setGoal(double goal) {
        pidController.setGoal(goal);
        setVoltage(pidController.calculate(encoder.getPosition())
                + feedforward.calculate(encoder.getPosition(), pidController.getSetpoint().velocity));

        currentGoal = goal;
        manualMode = false;
    }

    @Override
    public boolean atGoal() {
        return pidController.atGoal();
    }

    @Override
    public void zeroPosition() {
        encoder.setPosition(CLOSE_POS);
    }

    @Override
    public void resistGravity() {
        setVoltage(feedforward.calculate(encoder.getPosition(), 0));
    }

    @Override
    public double getAngle() {
        return encoder.getPosition();
    }

    @Override
    public void resetPID() {
        pidController.reset(encoder.getPosition(), encoder.getVelocity());
    }

    @Override
    public void resetPID(double newGoal) {
        if (newGoal - encoder.getPosition() > 0) {
            pidController.reset(encoder.getPosition(),
                    Math.max(encoder.getVelocity(), feedforward.calculate(encoder.getPosition(), 1)));
        } else {
            pidController.reset(encoder.getPosition(),
                    Math.min(encoder.getVelocity(), feedforward.calculate(encoder.getPosition(), 1)));
        }
    }

    @Override
    public void stayInCurrentGoal() {
        if (manualMode) {
            currentGoal = getAngle();
            resetPID();
        }
    }

    @Override
    public boolean getSensor() {
        return sensor.get();
    }

}
