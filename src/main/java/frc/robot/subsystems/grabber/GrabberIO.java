package frc.robot.subsystems.grabber;

public interface GrabberIO {
    public default void stop() {
    }

    public default void setSpeed(double speed) {
    }

    public default double getSpeed() {
        return 0.0;
    }

    public default void setVoltage(double volts) {
    }

    public default double getPosition() {
        return 0.0;
    }

    public default void setPosition(double position) {
    }

    public default double getVelocity() {
        return 0.0;
    }

    public default void resetEncoder() {
    }

    public default boolean getInverted() {
        return false;
    }

    public default void setInverted(boolean inverted) {
    }
}
