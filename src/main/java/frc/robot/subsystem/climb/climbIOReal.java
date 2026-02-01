package frc.robot.subsystem.climb;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.servohub.ServoHub;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.POM_lib.sensors.POMDigitalInput;

import static frc.robot.subsystem.climb.climbConstants.*;

public class climbIOReal implements climbIO {
    private final TalonFX motor;
    private final Servo servo;
    private final POMDigitalInput limitSwitch;
    private final MotorOutputConfigs configs;

    public climbIOReal() {
        motor = new TalonFX(MOTOR_ID);
        servo = new Servo(SERVO_CHANNEL);
        limitSwitch = new POMDigitalInput(LIMIT_SWITCH_CHANNEL, IS_NORAMLLY_OPEN);
        configs = new MotorOutputConfigs();
        configs.Inverted = InvertedValue.Clockwise_Positive;
        configs.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(configs); // should work
    }

    @Override
    public void updateInputs(climbIOInputs inputs) {
        inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.output = motor.getMotorOutputStatus().getValueAsDouble();
        inputs.angle = servo.getAngle();
        inputs.isPressed = isPressed();
    }

    @Override
    public void setMotorVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void stopMotor() {
        motor.stopMotor();
    }

    @Override
    public void setServoAngle(double angle) {
        servo.setAngle(angle);
    }

    @Override
    public boolean isPressed() {
        return limitSwitch.get();
    }
}
