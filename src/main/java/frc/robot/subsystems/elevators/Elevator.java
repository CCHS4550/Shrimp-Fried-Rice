package frc.robot.subsystems.elevators;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevators.ElevatorIO.ElevatorIOInputs;

public class Elevator extends SubsystemBase {
    public enum WantedState {
        IDLE,
        STOWED,
        L1,
        L2,
        L3,
        L4,
        POSITION,
        MANUAL,
        VOLTAGE
    }

    public enum SystemState {
        IDLE,
        STOWED,
        L1,
        L2,
        L3,
        L4,
        POSITION,
        MANUAL,
        VOLTAGE
    }

    private final ElevatorIO io;
    private final ElevatorIOInputs inputs = new ElevatorIOInputs();

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    private double targetPosition = 0.0;
    private double targetSpeed = 0.0;
    private double targetVoltage = 0.0;

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
            case IDLE:
                return SystemState.IDLE;
            case STOWED:
                return SystemState.STOWED;
            case L1:
                return SystemState.L1;
            case L2:
                return SystemState.L2;
            case L3:
                return SystemState.L3;
            case L4:
                return SystemState.L4;
            case POSITION:
                return SystemState.POSITION;
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
                io.stopElevator();
                break;
            case STOWED:
                targetPosition = ElevatorConstants.stowedPosition;
                io.setPosition(targetPosition);
                break;
            case L1:
                targetPosition = ElevatorConstants.l1Position;
                io.setPosition(targetPosition);
                break;
            case L2:
                targetPosition = ElevatorConstants.l2Position;
                io.setPosition(targetPosition);
                break;
            case L3:
                targetPosition = ElevatorConstants.l3Position;
                io.setPosition(targetPosition);
                break;
            case L4:
                targetPosition = ElevatorConstants.l4Position;
                io.setPosition(targetPosition);
                break;
            case POSITION:
                io.setPosition(targetPosition);
                break;
            case MANUAL:
                io.setSpeed(targetSpeed);
                break;
            case VOLTAGE:
                io.setVoltage(targetVoltage);
                break;
            default:
                io.stopElevator();
                break;
        }
    }

    public void setWantedState(WantedState wantedState) {
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
        targetVoltage = 0.0;
        io.stopElevator();
    }

    public void setSpeed(double speed) {
        targetSpeed = speed;
        wantedState = WantedState.MANUAL;
    }

    public void setVoltage(double volts) {
        targetVoltage = volts;
        wantedState = WantedState.VOLTAGE;
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
            () -> setWantedState(state),
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
