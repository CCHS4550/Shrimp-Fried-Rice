package frc.robot.subsystems.grabber;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    private final GrabberIO io;
    private final GrabberIOInputs inputs = new GrabberIOInputs();
    private final Alert grabberDisconnectedAlert = new Alert("Disconnected grabber spark", AlertType.kError);

    private WantedGrabberState wantedState = WantedGrabberState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    public Grabber(GrabberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        if (DriverStation.isDisabled()) {
            wantedState = WantedGrabberState.IDLE;
            systemState = SystemState.IDLE;
            io.stop();
        }

        systemState = handleStateTransitions();
        applyStates();

        Logger.recordOutput("Subsystems/Grabber/SystemState", systemState);
        Logger.recordOutput("Subsystems/Grabber/WantedState", wantedState);
        grabberDisconnectedAlert.set(!inputs.grabberConnected);
    }

    public SystemState handleStateTransitions() {
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

    public void applyStates() {
        switch (systemState) {
            case COLLECTING:
                io.setVoltage(3.0);
                break;
            case EJECTING:
                io.setVoltage(-3.0);
                break;
            case IDLE:
            default:
                io.stop();
                break;
        }
    }

    public void setWantedGrabberState(WantedGrabberState wantedState) {
        if (this.wantedState == wantedState) {
            return;
        }
        this.wantedState = wantedState;
        if (wantedState == WantedGrabberState.IDLE) {
            io.stop();
        }
    }

    public WantedGrabberState getWantedState() {
        return wantedState;
    }

    public SystemState getSystemState() {
        return systemState;
    }

    public void stop() {
        wantedState = WantedGrabberState.IDLE;
        io.stop();
    }

    public double getPositionRad() {
        return inputs.grabberPositionRad;
    }

    public double getVelocityRadPerSec() {
        return inputs.grabberVelocityRadPerSec;
    }

    public double getAppliedVolts() {
        return inputs.grabberAppliedVolts;
    }

    public double getCurrentAmps() {
        return inputs.grabberCurrentAmps;
    }

    public void resetEncoder() {
        io.resetEncoder();
    }

    public Command idleCommand() {
        return runOnce(this::stop);
    }
}