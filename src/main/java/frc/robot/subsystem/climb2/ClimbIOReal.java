package frc.robot.subsystem.climb2;

import static frc.robot.subsystem.climb2.ClimbConstants.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

public class ClimbIOReal implements ClimbIO {
    private final TalonFX climbMotor;
    private final Servo servo;
    private final POMSparkMax armMotor; 
    private RelativeEncoder encoder;
    private final POMDigitalInput limitSwitch;

    private TalonFXConfiguration climbConfig;
    private SparkMaxConfig armConfig;

    private final PositionVoltage preClimbRequest;
    private final PositionVoltage ClimbRequest;

    private PIDController pidController;
    
    private double currentGoal;
    
    private VoltageOut voltageOut;

    public ClimbIOReal() {
        climbMotor = new TalonFX(KRAKEN_ID);
        armMotor = new POMSparkMax(NEO_ID);
        encoder = armMotor.getEncoder();
        servo = new Servo(SERVO_CHANNEL);
        limitSwitch = new POMDigitalInput(LIMIT_SWITCH_CHANNEL, NORMALLY_OPEN);

        pidController = new PIDController(KpNeo, KiNeo, KdNeo);
        
        voltageOut = new VoltageOut(0);

        climbConfig = new TalonFXConfiguration();
        climbConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // need to check
        climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        climbConfig.Feedback.RotorToSensorRatio = gearRatio;
        climbConfig.TorqueCurrent.PeakForwardTorqueCurrent = currentLimit;
        climbConfig.TorqueCurrent.PeakReverseTorqueCurrent = -currentLimit;
        climbConfig.CurrentLimits.StatorCurrentLimit = currentLimit;
        climbConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        Slot0Configs preClimbSlot = new Slot0Configs().withKP(Kp).withKI(Ki).withKD(Kd).withKS(Ks)
                .withKG(BEFORE_CLIMB_Kg).withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        Slot1Configs ClimbSlot = new Slot1Configs().withKP(Kp).withKI(Ki).withKD(Kd).withKS(Ks).withKG(CLIMB_Kg)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        climbConfig.Slot0 = preClimbSlot;
        climbConfig.Slot1 = ClimbSlot;

        preClimbRequest = new PositionVoltage(0).withSlot(0);
        ClimbRequest = new PositionVoltage(0).withSlot(1);


        climbMotor.getConfigurator().apply(climbConfig);

        armConfig = new SparkMaxConfig();
        armConfig.idleMode(IdleMode.kCoast);

        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        servo.setAngle(CLOSE_SERVO_POS);
        resetPos();
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.krakenVoltage = climbMotor.getMotorVoltage().getValue();
        inputs.KrakenOutput = climbMotor.getMotorOutputStatus().getValueAsDouble();
        inputs.krakenPosition = climbMotor.getPosition().getValueAsDouble();
        inputs.krakenAtGoal = atKrakenGoal();

        inputs.neoVoltage = armMotor.getAppliedOutput();
        inputs.neoOutput = armMotor.getOutputCurrent();
        inputs.neoAngle = encoder.getPosition();

        inputs.servoAngle = servo.getAngle();

        inputs.isPressed = limitSwitch.get();
    }

    @Override
    public void resetPos() {
        if(isPressed()) {
            climbMotor.setPosition(0);
        }
    }

    @Override
    public void setKrakenVoltage(double voltage) {
        currentGoal = voltage;
        climbMotor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void stopKraken() {
        setKrakenVoltage(0);
        
    }

    @Override
    public void setNeoVoltage(double voltage) {
        armMotor.setVoltage(voltage);
    }

    @Override
    public void stopNeo() {
        setNeoVoltage(0);
    }

    @Override
    public void setServoAngle(double angle) {
        servo.setAngle(angle);
    }

    @Override
    public boolean isPressed(){
        return limitSwitch.get();
    }

    @Override
    public void preClimbGoToPos(double goal) {
        currentGoal = goal;
        climbMotor.setControl(preClimbRequest.withPosition(goal));
    }

    @Override
    public void ClimbGoToPos(double goal) {
        currentGoal = goal;
        climbMotor.setControl(ClimbRequest.withPosition(goal));
    }

    @Override
    public boolean atKrakenGoal() {
        return Math.abs(getPosition() - currentGoal) < TOLERANCE;
    }
    

    @Override
    public double getPosition() {
        return climbMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void setNeoGoal(double goal) {
        pidController.setSetpoint(goal);
        setNeoVoltage(pidController.calculate(armMotor.getEncoder().getPosition()));
    }

    @Override
    public boolean atNeoGoal() {
        return pidController.atSetpoint();
    }



}
