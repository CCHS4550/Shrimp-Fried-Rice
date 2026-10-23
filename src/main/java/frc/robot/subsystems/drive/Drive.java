package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    public final DriveIO io;

    public Drive(DriveIO io) {
        this.io = io;
    }

    public void stopRobot() {
        io.stopRobot();
    }

    public void driveStraight(double speed) {
        io.differentialDrive(speed, 0);
    }

    public void drive(double speed, double turn) {
        io.differentialDrive(speed, turn);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        io.tankDrive(leftSpeed, rightSpeed);
    }

    public void curvatureDrive(double speed, double turn, boolean allowTurnInPlace) {
        io.curvatureDrive(speed, turn, allowTurnInPlace);
    }

    public Command driveCommand(DoubleSupplier speed, DoubleSupplier turn) {
        return this.runEnd(
            () -> {
                double forwardSpeed = speed.getAsDouble();
                double turnSpeed = turn.getAsDouble();
                drive(forwardSpeed, turnSpeed);
            },
            this::stopRobot
        );
    }

    public Command basicDrive(double speed) {
        return this.runEnd(
            () -> driveStraight(speed),
            this::stopRobot
        );
    }

    public Command basicTurn(double turn) {
        return this.runEnd(
            () -> drive(0, turn),
            this::stopRobot
        );
    }

    public Command autoDriveForward(double speed, double time) {
        Command command = basicDrive(speed);
        return command.withTimeout(time);
    }

    public Command autoDriveTurn(double turn, double time) {
        Command command = basicTurn(turn);
        return command.withTimeout(time);
    }
}
