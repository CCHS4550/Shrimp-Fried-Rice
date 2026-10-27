package frc.robot.subsystems.drive;

public interface DriveIO {
    public default void stopRobot() {
    }

    public default void differentialDrive(double forwardSpeed, double turnSpeed) {
    }

    public default void tankDrive(double leftSpeed, double rightSpeed) {
    }

    public default void curvatureDrive(double forwardSpeed, double turnSpeed, boolean allowTurnInPlace) {
    }
}
