package frc.robot.subsystems.grabber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;
import frc.robot.subsystems.grabber.GrabberIO.GrabberIOInputs;

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

    private WantedGrabberState wantedState = WantedGrabberState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    private double targetSpeed = 0.0;
    private double targetVoltage = 0.0;

    public Grabber(GrabberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        systemState = handleStateTransitions();
        applyStates();
    }

    private SystemState handleStateTransitions() {
        switch (wantedState) {
            case IDLE:
                return SystemState.IDLE;
            case COLLECTING:
                return SystemState.COLLECTING;
            case EJECTING:
                return SystemState.EJECTING;
            case MANUAL:
                return SystemState.MANUAL;
            case VOLTAGE:
                return SystemState.VOLTAGE;
            default:
                return SystemState.IDLE;
        }
    }

    private void applyStates() {
        switch (systemState) {
            case IDLE:
                io.stop();
                break;
            case COLLECTING:
                io.setVoltage(GrabberConstants.collectVoltage);
                break;
            case EJECTING:
                io.setVoltage(GrabberConstants.ejectVoltage);
                break;
            case MANUAL:
                io.setSpeed(targetSpeed);
                break;
            case VOLTAGE:
                io.setVoltage(targetVoltage);
                break;
            default:
                io.stop();
                break;
        }
    }

    public void setWantedState(WantedGrabberState wantedState) {
        this.wantedState = wantedState;
    }

    public WantedGrabberState getWantedState() {
        return wantedState;
    }

    public SystemState getSystemState() {
        return systemState;
    }

    public void stop() {
        wantedState = WantedGrabberState.IDLE;
        targetSpeed = 0.0;
        targetVoltage = 0.0;
        io.stop();
    }

    public void setSpeed(double speed) {
        targetSpeed = speed;
        wantedState = WantedGrabberState.MANUAL;
    }

    public void setVoltage(double volts) {
        targetVoltage = volts;
        wantedState = WantedGrabberState.VOLTAGE;
    }

    public double getPosition() {
        return inputs.position;
    }

    public double getVelocity() {
        return inputs.velocityRPM;
    }

    public double getAppliedOutput() {
        return inputs.appliedOutput;
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

    public Command collectCommand() {
        return setWantedStateCommand(WantedGrabberState.COLLECTING);
    }

    public Command ejectCommand() {
        return setWantedStateCommand(WantedGrabberState.EJECTING);
    }
}
