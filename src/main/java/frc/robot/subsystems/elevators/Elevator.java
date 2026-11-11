package frc.robot.subsystems.elevators;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevators.ElevatorIO.ElevatorIOInputs;

public class Elevator extends SubsystemBase {
    public enum WantedState {
        IDLE,
        POSITION,
        MANUAL
    }

    public enum SystemState {
        IDLE,
        POSITION,
        MANUAL
    }

    private final ElevatorIO io;
    private final ElevatorIOInputs inputs = new ElevatorIOInputs();

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    private double targetPosition = 0.0;
    private double targetSpeed = 0.0;

    public Elevator(ElevatorIO io) {
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
            case POSITION:
                return SystemState.POSITION;
            case MANUAL:
                return SystemState.MANUAL;
            case IDLE:
            default:
                return SystemState.IDLE;
        }
    }

    private void applyStates() {
        switch (systemState) {
            case POSITION:
                io.setPosition(targetPosition);
                break;
            case MANUAL:
                io.setSpeed(targetSpeed);
                break;
            case IDLE:
            default:
                io.stopElevator();
                break;
        }
    }

    public void setWantedElevatorState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public WantedState getWantedState() {
        return wantedState;
    }

    public SystemState getSystemState() {
        return systemState;
    }

    public boolean atSetpoint() {
        return MathUtil.isNear(targetPosition, inputs.position, ElevatorConstants.positionTolerance);
    }

    public void stopElevator() {
        wantedState = WantedState.IDLE;
        targetSpeed = 0.0;
        io.stopElevator();
    }

    public void setSpeed(double speed) {
        targetSpeed = speed;
        wantedState = WantedState.MANUAL;
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void setPosition(double position) {
        targetPosition = position;
        wantedState = WantedState.POSITION;
    }

    public double getPosition() {
        return inputs.position;
    }

    public double getVelocity() {
        return inputs.velocity;
    }

    public void resetEncoder() {
        io.resetEncoder();
    }

    public Command setStateCommand(WantedState state) {
        return runEnd(
            () -> setWantedElevatorState(state),
            this::stopElevator
        );
    }

    public Command idleCommand() {
        return runOnce(this::stopElevator);
    }

    public Command runElevator(double speed) {
        return runEnd(
            () -> setSpeed(speed),
            this::stopElevator
        );
    }

    public Command moveToPosition(double position) {
        return runEnd(
            () -> setPosition(position),
            this::stopElevator
        );
    }
}