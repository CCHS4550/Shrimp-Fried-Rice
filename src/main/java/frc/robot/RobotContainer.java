// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.controlschemes.DriveScheme;
import frc.robot.subsystems.swerve.SwerveHardware;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.controlschemes.GrabberScheme;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.grabber.GrabberIO;
import frc.robot.subsystems.grabber.GrabberIOHardware;


/**
 * WPILib recommends putting most robot logic in here. Creek takes it 1 step further and abstracts it to seperate files
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...  
  private final SwerveIO swerveHardware = new SwerveHardware();
  private final Swerve drive = new Drive(swerveHardware);
  private final GrabberIO grabberIOHardware = new GrabberIOHardware();
  private final Grabber grabber = new Grabber(grabberIOHardware);
  private final GrabberScheme grabberScheme = new GrabberScheme();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
      DriveScheme.configure(drive, 1);
      grabberScheme.configure(grabber, 1);

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
