// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  
  public static class DriveConstants{
    public static final int FLdeviceID = 999;
    public static final int FRdeviceID = 999;
    public static final int BLdeviceID = 999;
    public static final int BRdeviceID = 999;
  }

  public static class GrabberConstants {
    // Replace with the CAN ID assigned to the grabber motor.
    public static final int motorDeviceID = 999;
    public static final double motorSpeed = 0.75;
  }

  public static class ElevatorConstants {
    public static final int motorDeviceID = 999;
    public static final double supplyCurrentLimit = 40.0;
    public static final double statorCurrentLimit = 80.0;
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kG = 0.0;
    public static final double positionTolerance = 0.5;
    public static final double stowedPosition = 0.0;
    public static final double l1Position = 10.0;
    public static final double l2Position = 20.0;
    public static final double l3Position = 30.0;
    public static final double l4Position = 40.0;
  }
}
