package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.elevators.Elevator;

public class Superstructure extends SubsystemBase {
    public boolean stopApplyingState = false;

    private final SwerveSubsystem swerveSubsystem;
    private final Elevator elevator;
    private final Grabber grabber;

    @AutoLogOutput private WantedSuperstructureState wantedState1 = WantedSuperstructureState.IDLE;
    @AutoLogOutput private SystemState systemState = SystemState.IDLE;

    public Superstructure(SwerveSubsystem swerveSubsystem, Elevator elevator, Grabber grabber){
        this.swerveSubsystem = swerveSubsystem;
        this.elevator = elevator;
        this.grabber = grabber;
    }

    @Ovverride
    public void periodic(){
        
    }
}
