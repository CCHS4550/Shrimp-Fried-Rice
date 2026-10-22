// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.controlschemes.DriveScheme;
import frc.robot.subsystems.drive.DriveIOHardware;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.controlschemes.GrabberScheme;
import frc.robot.subsystems.grabber.Grabber;


/**
 * WPILib recommends putting most robot logic in here. Creek takes it 1 step further and abstracts it to seperate files
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...  
  private final DriveScheme driveScheme = new DriveScheme();
  private final DriveIO driveIOHardware = new DriveIOHardware();
  private final Drive drive = new Drive(driveIOHardware);
  private final Grabber grabber = new Grabber();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
      driveScheme.configure(drive, 1);
      new GrabberScheme().configure(grabber, 1);

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
