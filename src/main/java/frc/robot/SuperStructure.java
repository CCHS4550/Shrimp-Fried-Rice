package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem.SystemState;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.elevators.Elevator;

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

    @AutoLogOutput private WantedSuperstructureState wantedState1 = WantedSuperstructureState.IDLE;
    @AutoLogOutput private SystemState systemState = SystemState.IDLE;

    public Superstructure(SwerveSubsystem swerveSubsystem, Elevator elevator, Grabber grabber){
        this.swerveSubsystem = swerveSubsystem;
        this.elevator = elevator;
        this.grabber = grabber;
    }

    @Override
    public void periodic(){
       
    }

    public void setWantedSuperStructure(WantedSuperstructureState wantedState){
        this.wantedState1 = wantedState;
    }

    private void applyState(){
        switch (systemState){
            case IDLE:

            case INTAKING:
                elevator.
        }
    }

}
