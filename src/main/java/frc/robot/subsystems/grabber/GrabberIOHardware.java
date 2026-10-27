package frc.robot.subsystems.grabber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.GrabberConstants;
import frc.robot.SparkMaxMotorController;

public class GrabberIOHardware implements GrabberIO {
    private final SparkMaxMotorController motor;

    public GrabberIOHardware() {
        this(GrabberConstants.motorDeviceID, false, IdleMode.kBrake);
    }

    public GrabberIOHardware(int deviceID, boolean inverted, IdleMode idleMode) {
        motor = new SparkMaxMotorController(
            "GrabberMotor",
            deviceID,
            MotorType.kBrushless,
            inverted,
            idleMode
        );
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setSpeed(double speed) {
        double clampedSpeed = MathUtil.clamp(speed, -1.0, 1.0);
        motor.set(clampedSpeed);
    }

    @Override
    public double getSpeed() {
        return motor.get();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public double getPosition() {
        return motor.getPosition();
    }

    @Override
    public void setPosition(double position) {
        motor.setPosition(position);
    }

    @Override
    public double getVelocity() {
        return motor.getVelocity();
    }

    @Override
    public void resetEncoder() {
        motor.resetEncoder();
    }

    @Override
    public boolean getInverted() {
        return motor.getInverted();
    }

    @Override
    public void setInverted(boolean inverted) {
        motor.setInverted(inverted);
    }
}
