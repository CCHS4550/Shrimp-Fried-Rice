package frc.robot.subsystems.grabber;

public interface GrabberIO {
    public static class GrabberIOInputs {
        public boolean grabberConnected = false;
        public double grabberPositionRad = 0.0;
        public double grabberVelocityRadPerSec = 0.0;
        public double grabberAppliedVolts = 0.0;
        public double grabberCurrentAmps = 0.0;
    }

    public default void updateInputs(GrabberIOInputs inputs) {
    }

    public default void setVoltage(double volts) {
    }

    public default void setSpeed(double speed) {
    }

    public default void stop() {
    }

    public default void resetEncoder() {
    }
}
