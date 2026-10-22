package frc.robot.controlschemes;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.grabber.Grabber;

/** Xbox trigger bindings for collecting and ejecting a game piece. */
public class GrabberScheme {
    public void configure(Grabber grabber, int port) {
        CommandXboxController controller = new CommandXboxController(port);

        // Right trigger collects; left trigger reverses the chain to eject.
        controller.rightTrigger().whileTrue(
            grabber.runAtSpeed(Constants.GrabberConstants.motorSpeed)
        );
        controller.leftTrigger().whileTrue(
            grabber.runAtSpeed(-Constants.GrabberConstants.motorSpeed)
        );
    }
}
