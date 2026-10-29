package frc.robot.subsystems.grabber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.GrabberConstants;
import frc.robot.SparkMaxMotorController;

public class GrabberIOHardware implements GrabberIO {
    private final SparkMaxMotorController motor = new SparkMaxMotorController(
        "GrabberMotor",
        GrabberConstants.motorDeviceID,
        MotorType.kBrushless,
        false,
        IdleMode.kBrake
    );

    @Override
    public void updateInputs(GrabberIOInputs inputs) {
        inputs.appliedOutput = motor.get();
        inputs.velocityRPM = motor.getVelocity();
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
