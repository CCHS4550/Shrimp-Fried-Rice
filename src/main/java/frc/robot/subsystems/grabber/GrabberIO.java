package frc.robot.subsystems.grabber;

public interface GrabberIO {
    public static class GrabberIOInputs {
        public double appliedOutput = 0.0;
        public double velocityRPM = 0.0;
    }

    public default void updateInputs(GrabberIOInputs inputs) {
    }

    public default void stop() {
    }

    public default void setSpeed(double speed) {
    }

    public default void setVoltage(double volts) {
    }
}
