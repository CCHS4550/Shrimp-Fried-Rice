package frc.robot.subsystems.grabber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;
import frc.robot.subsystems.grabber.GrabberIO.GrabberIOInputs;

public class Grabber extends SubsystemBase {
    private final GrabberIO io;
    private final GrabberIOInputs inputs = new GrabberIOInputs();

    public Grabber(GrabberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void stop() {
        io.stop();
    }

    public void setSpeed(double speed) {
        io.setSpeed(speed);
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public Command runAtSpeed(double speed) {
        return runEnd(
            () -> setSpeed(speed),
            this::stop
        );
    }

    public Command runVoltage(double volts) {
        return runEnd(
            () -> setVoltage(volts),
            this::stop
        );
    }

    public Command grab() {
        return runAtSpeed(GrabberConstants.motorSpeed);
    }

    public Command eject() {
        return runAtSpeed(-GrabberConstants.motorSpeed);
    }
}
