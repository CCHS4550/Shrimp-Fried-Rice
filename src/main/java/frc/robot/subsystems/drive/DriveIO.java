package frc.robot.subsystems.drive;

public interface DriveIO {
    public static class DriveIOInputs {
        public double leftPosition = 0.0;
        public double leftVelocity = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftSupplyCurrentAmps = 0.0;
        public double leftStatorCurrentAmps = 0.0;
        public double rightPosition = 0.0;
        public double rightVelocity = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightSupplyCurrentAmps = 0.0;
        public double rightStatorCurrentAmps = 0.0;
    }

    public default void updateInputs(DriveIOInputs inputs) {
    }

    public default void stopRobot() {
    }

    public default void differentialDrive(double forwardSpeed, double turnSpeed) {
    }

    public default void tankDrive(double leftSpeed, double rightSpeed) {
    }

    public default void curvatureDrive(double forwardSpeed, double turnSpeed, boolean allowTurnInPlace) {
    }
}
