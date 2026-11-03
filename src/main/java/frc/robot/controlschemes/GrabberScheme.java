package frc.robot.controlschemes;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.grabber.Grabber.WantedGrabberState;

public class GrabberScheme {
    public void configure(Grabber grabber, int port) {
        CommandXboxController controller = new CommandXboxController(port);

        Trigger rightTrigger = controller.rightTrigger();
        Command collectCommand = grabber.setWantedStateCommand(WantedGrabberState.COLLECTING);
        rightTrigger.whileTrue(collectCommand);

        Trigger leftTrigger = controller.leftTrigger();
        Command ejectCommand = grabber.setWantedStateCommand(WantedGrabberState.EJECTING);
        leftTrigger.whileTrue(ejectCommand);
    }
}
