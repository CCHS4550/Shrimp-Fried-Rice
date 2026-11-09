package frc.robot.Config;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.util.Units;
import frc.robot.Util.CanDeviceID;

public class RobotConfig {
    private static final CANBus CANIVORE_CANBUS  = new CANBus("CANivore");

    public final CanDeviceID FRONT_LEFT_DRIVE_MOTOR = new CanDeviceID(1);
    public final CanDeviceID FRONT_LEFT_STEER_MOTOR = new CanDeviceID(2);
    public final CanDeviceID FRONT_LEFT_STEER_ENCODER = new CanDeviceID(3);

    public final CanDeviceID FRONT_RIGHT_DRIVE_MOTOR = new CanDeviceID(4);
    public final CanDeviceID FRONT_RIGHT_STEER_MOTOR = new CanDeviceID(5);
    public final CanDeviceID FRONT_RIGHT_STEER_ENCODER = new CanDeviceID(6);

    public final CanDeviceID BACK_LEFT_DRIVE_MOTOR = new CanDeviceID(7);
    public final CanDeviceID BACK_LEFT_STEER_MOTOR = new CanDeviceID(8);
    public final CanDeviceID BACK_LEFT_STEER_ENCODER = new CanDeviceID(9);

    public final CanDeviceID BACK_RIGHT_DRIVE_MOTOR = new CanDeviceID(10);
    public final CanDeviceID BACK_RIGHT_STEER_MOTOR = new CanDeviceID(11);
    public final CanDeviceID BACK_RIGHT_STEER_ENCODER = new CanDeviceID(12);

    public final CanDeviceID ELEVATION_MOTOR = new CanDeviceID(13, CANIVORE_CANBUS);

    /**
     * Wheel radius in meters. Accuracy in these measurements affects wheel odometry which measures
     * distance as a function of the number of rotations * wheel circumference.
     */
    private final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2);

    /** Ratio between the drive motor shaft and the output shaft the wheel is mounted on. */
    private final double DRIVE_GEAR_RATIO = 5.27;

    /** Ratio between the steer motor shaft and the steer output shaft. */
    private final double STEER_GEAR_RATIO = 26;

    //arm gear ratio
    private final double ARM_GEAR_RATIO = 80;
}
