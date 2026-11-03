package frc.robot.subsystems.grabber;

public interface GrabberIO {
    public static class GrabberIOInputs {
        public double grabberVoltage = 0.0;
        public double grabberPositionRad = 0.0;
        public double grabberSupplyCurrent = 0.0;
        public double grabberStatorCurrent = 0.0;
        public double grabberTemperature = 0.0;
        public double grabberVelocityRadPerSec = 0.0;
        public double grabberAccelRadPerSecSquared = 0.0;
    }

    public default void updateInputs(GrabberIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void setSpeed(double speed) {
    }

    public default void stop() {
    }
}
