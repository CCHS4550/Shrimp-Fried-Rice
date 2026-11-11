package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem.SystemState;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.grabber.Grabber.WantedGrabberState;
import frc.robot.subsystems.elevators.Elevator;
import frc.robot.subsystems.elevators.Elevator.WantedState;

public class Superstructure extends SubsystemBase {
    public boolean stopApplyingState = false;

    private final SwerveSubsystem swerveSubsystem;
    private final Elevator elevator;
    private final Grabber grabber;

    public enum WantedSuperstructureState {
        IDLE,
        INTAKING,
        SCORE
    }

    public enum SystemState {
        IDLE,
        INTAKING,
        SCORE
    }

    @AutoLogOutput private WantedSuperstructureState wantedState1 = WantedSuperstructureState.IDLE;
    @AutoLogOutput private SystemState systemState = SystemState.IDLE;

    public Superstructure(SwerveSubsystem swerveSubsystem, Elevator elevator, Grabber grabber){
        this.swerveSubsystem = swerveSubsystem;
        this.elevator = elevator;
        this.grabber = grabber;
    }

    @Override
    public void periodic(){
       systemState = handleStateTransition();

       applyState();
    }

    public void setWantedSuperStructure(WantedSuperstructureState wantedState){
        this.wantedState1 = wantedState;
    }

    private void applyState(){
        switch (systemState){
            case IDLE:
                elevator.setWantedElevatorState(WantedState.IDLE);
            case INTAKING:
                elevator.setWantedElevatorState(WantedState.);
                grabber.setWantedGrabberState(WantedGrabberState.COLLECTING);
            case SCORE:
                elevator.setWantedElevatorState(WantedState.POSITION);
                if(/*condition that make sure elevator is in the right position before ejecting*/){
                    grabber.setWantedGrabberState(WantedGrabberState.EJECTING);
                }
        }
    }

    private SystemState handleStateTransitions(){
        switch (wantedState1) {
            case IDLE:
                return SystemState.IDLE;
            case INTAKING:
                return SystemState.INTAKING;
            case SCORE:
                return SystemState.SCORE;
            default:
                return SystemState.IDLE;
        }
    }

}
