package frc.robot.subsystems.grabber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SparkMaxMotorController;

/** Controls the chain-driven pair of grabber wheels with one motor. */
public class Grabber extends SubsystemBase {
    private final SparkMaxMotorController motor = new SparkMaxMotorController(
        "GrabberMotor",
        Constants.GrabberConstants.motorDeviceID,
        MotorType.kBrushless,
        false,
        IdleMode.kBrake
    );

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.stopMotor();
    }

    /** Runs the grabber at the requested speed and stops it when interrupted or ended. */
    public Command runAtSpeed(double speed) {
        return this.runEnd(() -> setSpeed(speed), this::stop);
    }
}
