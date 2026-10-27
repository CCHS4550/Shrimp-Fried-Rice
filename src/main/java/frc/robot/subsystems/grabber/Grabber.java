package frc.robot.subsystems.grabber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {
    private final GrabberIO io;

    public Grabber(GrabberIO io) {
        this.io = io;
    }

    public void setSpeed(double speed) {
        io.setSpeed(speed);
    }

    public void stop() {
        io.stop();
    }

    public Command runAtSpeed(double speed) {
        return this.runEnd(() -> setSpeed(speed), this::stop);
    }
}
