package frc.robot.subsystems.grabber;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;

public class Grabber extends SubsystemBase {
    public final GrabberIO io;

    public Grabber(GrabberIO io) {
        this.io = io;
    }

    public void stop() {
        io.stop();
    }

    public void setSpeed(double speed) {
        io.setSpeed(speed);
    }

    public double getSpeed() {
        return io.getSpeed();
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public double getPosition() {
        return io.getPosition();
    }

    public void setPosition(double position) {
        io.setPosition(position);
    }

    public double getVelocity() {
        return io.getVelocity();
    }

    public void resetEncoder() {
        io.resetEncoder();
    }

    public boolean getInverted() {
        return io.getInverted();
    }

    public void setInverted(boolean inverted) {
        io.setInverted(inverted);
    }

    public void grab() {
        setSpeed(GrabberConstants.motorSpeed);
    }

    public void eject() {
        setSpeed(-GrabberConstants.motorSpeed);
    }

    public Command runAtSpeed(double speed) {
        return this.runEnd(
            () -> setSpeed(speed),
            this::stop
        );
    }

    public Command runAtSpeed(DoubleSupplier speed) {
        return this.runEnd(
            () -> {
                double targetSpeed = speed.getAsDouble();
                setSpeed(targetSpeed);
            },
            this::stop
        );
    }

    public Command runVoltage(double volts) {
        return this.runEnd(
            () -> setVoltage(volts),
            this::stop
        );
    }

    public Command grabCommand() {
        return runAtSpeed(GrabberConstants.motorSpeed);
    }

    public Command ejectCommand() {
        return runAtSpeed(-GrabberConstants.motorSpeed);
    }

    public Command autoRunAtSpeed(double speed, double time) {
        Command command = runAtSpeed(speed);
        return command.withTimeout(time);
    }
}
