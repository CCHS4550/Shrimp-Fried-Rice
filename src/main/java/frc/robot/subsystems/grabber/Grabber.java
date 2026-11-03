package frc.robot.subsystems.grabber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;
import frc.robot.subsystems.grabber.GrabberIO.GrabberIOInputs;
import org.littletonrobotics.junction.Logger;

public class Grabber extends SubsystemBase {
    public enum WantedGrabberState {
        IDLE,
        COLLECTING,
        EJECTING
    }

    public enum SystemState {
        IDLE,
        COLLECTING,
        EJECTING
    }

    private final GrabberIO grabberIO;
    private final GrabberIOInputs inputs = new GrabberIOInputs();

    private WantedGrabberState wantedState = WantedGrabberState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    public Grabber(GrabberIO grabberIO) {
        this.grabberIO = grabberIO;
    }

    @Override
    public void periodic() {
        grabberIO.updateInputs(inputs);
        systemState = handleStateTransitions();
        Logger.recordOutput("Subsystems/Grabber/SystemState", systemState);
        Logger.recordOutput("Subsystems/Grabber/DesiredState", wantedState);
        applyStates();
    }

    private SystemState handleStateTransitions() {
        switch (wantedState) {
            case COLLECTING:
                return SystemState.COLLECTING;
            case EJECTING:
                return SystemState.EJECTING;
            case IDLE:
            default:
                return SystemState.IDLE;
        }
    }

    private void applyStates() {
        switch (systemState) {
            case COLLECTING:
                grabberIO.setVoltage(GrabberConstants.collectVoltage);
                break;
            case EJECTING:
                grabberIO.setVoltage(GrabberConstants.ejectVoltage);
                break;
            case IDLE:
            default:
                grabberIO.setVoltage(0.0);
                break;
        }
    }

    public void setWantedState(WantedGrabberState state) {
        if (state == this.wantedState) {
            return;
        }
        this.wantedState = state;
    }

    public WantedGrabberState getWantedState() {
        return wantedState;
    }

    public SystemState getSystemState() {
        return systemState;
    }

    public Command setWantedStateCommand(WantedGrabberState state) {
        return runEnd(
            () -> setWantedState(state),
            () -> setWantedState(WantedGrabberState.IDLE)
        );
    }

    public Command collectCommand() {
        return setWantedStateCommand(WantedGrabberState.COLLECTING);
    }

    public Command ejectCommand() {
        return setWantedStateCommand(WantedGrabberState.EJECTING);
    }
}
