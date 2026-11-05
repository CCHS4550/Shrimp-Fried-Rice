package frc.robot.subsystems.grabber;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;
import frc.robot.subsystems.grabber.GrabberIO.GrabberIOInputs;
import org.littletonrobotics.junction.Logger;

public class Grabber extends SubsystemBase {
    public enum WantedGrabberState {
        IDLE,
        COLLECTING,
        EJECTING,
        MANUAL,
        VOLTAGE
    }

    public enum SystemState {
        IDLE,
        COLLECTING,
        EJECTING,
        MANUAL,
        VOLTAGE
    }

    private final GrabberIO io;
    private final GrabberIOInputs inputs = new GrabberIOInputs();
    private final Alert grabberDisconnectedAlert = new Alert("Disconnected grabber spark", AlertType.kError);

    private WantedGrabberState wantedState = WantedGrabberState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    private double manualSpeed = 0.0;
    private double manualVoltage = 0.0;

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
            case MANUAL:
                return SystemState.MANUAL;
            case VOLTAGE:
                return SystemState.VOLTAGE;
            case IDLE:
            default:
                return SystemState.IDLE;
        }
    }

    public void applyStates() {
        switch (systemState) {
            case COLLECTING:
                io.setVoltage(GrabberConstants.collectVoltage);
                break;
            case EJECTING:
                io.setVoltage(GrabberConstants.ejectVoltage);
                break;
            case MANUAL:
                io.setSpeed(manualSpeed);
                break;
            case VOLTAGE:
                io.setVoltage(manualVoltage);
                break;
            case IDLE:
            default:
                io.stop();
                break;
        }
    }

    public void setWantedState(WantedGrabberState wantedState) {
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
        manualSpeed = 0.0;
        manualVoltage = 0.0;
        io.stop();
    }

    public void setSpeed(double speed) {
        manualSpeed = speed;
        wantedState = WantedGrabberState.MANUAL;
    }

    public void setVoltage(double volts) {
        manualVoltage = volts;
        wantedState = WantedGrabberState.VOLTAGE;
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

    public Command setWantedStateCommand(WantedGrabberState state) {
        return runEnd(
            () -> setWantedState(state),
            this::stop
        );
    }

    public Command idleCommand() {
        return runOnce(this::stop);
    }

    public Command collectCommand() {
        return setWantedStateCommand(WantedGrabberState.COLLECTING);
    }

    public Command ejectCommand() {
        return setWantedStateCommand(WantedGrabberState.EJECTING);
    }

    public Command runAtSpeed(double speed) {
        return runEnd(
            () -> setSpeed(speed),
            this::stop
        );
    }

    public Command runVoltage(double volts) {
        return runEnd(
            () -> setVoltage(volts),
            this::stop
        );
    }
}
