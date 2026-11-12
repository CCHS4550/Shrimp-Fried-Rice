package frc.robot.subsystems.elevators;

public interface ElevatorIO {
    public static class ElevatorIOInputs {
        public boolean connected = false;
        public double position = 0.0;
        public double velocity = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    public default void stopElevator() {
    }

    public default void setSpeed(double speed) {
    }

    public default void setVoltage(double volts) {
    }

    public default void setPosition(double position) {
    }

    public default double getPosition() {
        return 0.0;
    }

    public default double getVelocity() {
        return 0.0;
    }

    public default void resetEncoder() {
    }
}
