package frc.robot.subsystems.shooterArm;

import static frc.robot.subsystems.shooterArm.ShooterArmConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.POM_lib.Motors.POMSparkMax;

public class ShooterArmIOReal implements ShooterArmIO {

    private POMSparkMax motor;
    private RelativeEncoder encoder;
    private ProfiledPIDController pidController;
    private final ArmFeedforward feedforward;
    private double currentGoal = 0.0;
    private boolean manualMode = false;
    private SparkMaxConfig config;

    public ShooterArmIOReal() {
        motor = new POMSparkMax(MOTOR_ID);
        encoder = motor.getEncoder();
        
        pidController = new ProfiledPIDController(kp, ki, kd, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        feedforward = new ArmFeedforward(ks, kv, kg);
        
        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake).inverted(false)
                .smartCurrentLimit(currentLimit)
                .voltageCompensation(voltageCompensation);

        config.encoder.positionConversionFactor(gearRatio)
                .velocityConversionFactor(velocityConversionFactor);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        zeroPosition();

    }

    @Override
    public void updateInputs(ShooterArmIOInputs inputs) {
        inputs.motorConnected = motor.getFirmwareVersion() != 0;
        inputs.armPosition = encoder.getPosition();
        inputs.armVelocity = encoder.getVelocity();
        inputs.motorVoltage = motor.getBusVoltage();
        inputs.motorAppliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.set(voltage);
    }

    @Override
    public void stopMotor() {
        motor.stop();
    }

    @Override
    public void setGoal(double goal) {
        pidController.setGoal(goal);
        setVoltage(pidController.calculate(encoder.getPosition())
                +feedforward.calculate(encoder.getPosition(), pidController.getSetpoint().velocity));

        currentGoal = goal;
        manualMode = false;
    }

    @Override
    public boolean atGoal() {
        return pidController.atGoal();
    }

    @Override
    public void zeroPosition() {
        encoder.setPosition(0.0);
    }

    @Override
    public void resistGravity() {
        setVoltage(feedforward.calculate(encoder.getPosition(), 0));
    }

    @Override
    public double getPos() {
        return encoder.getPosition();
    }

    @Override
    public void resetPID() {
        pidController.reset(encoder.getPosition(), encoder.getVelocity());
    }

    @Override
    public void resetPID(double newGoal) {
        if (newGoal - encoder.getPosition() > 0) {
            pidController.reset(encoder.getPosition(), Math.max(encoder.getVelocity(), feedforward.calculate(encoder.getPosition(), 1)));
        } else {
            pidController.reset(encoder.getPosition(), Math.min(encoder.getVelocity(),feedforward.calculate(encoder.getPosition(), 1)));
        }
    }

    @Override
    public void stayInCurrentGoal() {
        if (manualMode) {
            currentGoal = getPos();
            resetPID();
        }
    }
    
}
