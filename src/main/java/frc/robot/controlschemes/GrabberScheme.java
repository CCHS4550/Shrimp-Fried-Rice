package frc.robot.controlschemes;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.grabber.Grabber.WantedGrabberState;

public class GrabberScheme {
    private CommandXboxController controller;

    public void configure(Grabber grabber, int port) {
        controller = new CommandXboxController(port);
        configureButtons(grabber);
    }

    private void configureButtons(Grabber grabber) {
        RunCommand defaultIdleCommand = new RunCommand(
            () -> grabber.setWantedState(WantedGrabberState.IDLE),
            grabber
        );
        grabber.setDefaultCommand(defaultIdleCommand);

        Trigger rightTrigger = controller.rightTrigger();
        Trigger leftTrigger = controller.leftTrigger();

        Trigger leftInactive = leftTrigger.negate();
        Trigger rightInactive = rightTrigger.negate();

        Trigger collectTrigger = rightTrigger.and(leftInactive);
        Trigger ejectTrigger = leftTrigger.and(rightInactive);
        Trigger idleTrigger = rightInactive.and(leftInactive);

        Command collectCommand = grabber.setWantedStateCommand(WantedGrabberState.COLLECTING);
        collectTrigger.whileTrue(collectCommand);

        Command ejectCommand = grabber.setWantedStateCommand(WantedGrabberState.EJECTING);
        ejectTrigger.whileTrue(ejectCommand);

        Command idleCommand = grabber.idleCommand();
        idleTrigger.onTrue(idleCommand);
    }
}
