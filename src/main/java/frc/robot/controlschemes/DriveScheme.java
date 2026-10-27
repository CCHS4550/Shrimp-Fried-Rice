package frc.robot.controlschemes;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;

public class DriveScheme {
    private static CommandXboxController controller;
    private static DoubleSupplier driveSpeed = () -> 1.0;

    private DriveScheme() {
    }

    public static void configure(Drive driveTrain, int port) {
        controller = new CommandXboxController(port);

        configureButtons(driveTrain);
    }

    private static void configureButtons(Drive drivetrain) {
        RunCommand drive = new RunCommand(() -> {
            double speedMultiplier = driveSpeed.getAsDouble();
            double forward = controller.getLeftY() * speedMultiplier;
            double turn = controller.getRightX() * speedMultiplier;
            drivetrain.drive(forward, turn);
        }, drivetrain);

        drivetrain.setDefaultCommand(drive);
    }
}
