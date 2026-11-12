// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Config.RobotConfig;
import frc.robot.Superstructure.WantedSuperstructureState;
import frc.robot.controlschemes.DriveScheme;
import frc.robot.subsystems.swerve.SwerveIOCTRE;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.controlschemes.GrabberScheme;
import frc.robot.subsystems.elevators.Elevator;
import frc.robot.subsystems.elevators.ElevatorIOHardware;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.grabber.GrabberIO;
import frc.robot.subsystems.grabber.GrabberIOHardware;


/**
 * WPILib recommends putting most robot logic in here. Creek takes it 1 step further and abstracts it to separate files
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...  
  private final Elevator elevator;
  private final Grabber grabber;
  private final SwerveSubsystem swerveSubsystem;
  private final Superstructure superstructure;

  private final CommandXboxController controller = new CommandXboxController(0);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    //DriveScheme.configure(drive, 1);
    //grabberScheme.configure(grabber, 1);
    RobotConfig config = new RobotConfig();

    swerveSubsystem = new SwerveSubsystem(new SwerveIOCTRE(config.getSwerveDrivetrainConstants(), config.getModuleConstants()));

    elevator = new Elevator(new ElevatorIOHadware(config));
    grabber = new Grabber(new GrabberIOHardware(config));


    superstructure = new Superstructure(swerveSubsystem, elevator, grabber);


    controller
        .x()
        .onTrue(
          new InstantCommand(
            () -> superstructure.setWantedSuperStructure(WantedSuperstructureState.INTAKING)))
        .onFalse(
          new InstantCommand(
            () -> superstructure.setWantedSuperStructure(WantedSuperstructureState.IDLE)));

            
    controller
        .y()
        .onTrue(
          new InstantCommand(
            () -> superstructure.setWantedSuperStructure(WantedSuperstructureState.SCORE)))
        .onFalse(new InstantCommand(
          () -> superstructure.setWantedSuperStructure(WantedSuperstructureState.IDLE)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Command() {
      
    };
  }
}
