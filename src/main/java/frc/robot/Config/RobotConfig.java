package frc.robot.Config;

import com.ctre.phoenix6.CANBus;
import frc.robot.Util.CanDeviceID;

public class RobotConfig {
    private static final CANBus CANIVORE_CANBUS  = new CANBus("CANivore");

    public final CanDeviceID FRONT_LEFT_DRIVE_MOTOR = new CanDeviceID(0);
    public final CanDEviceID FRONT_RIGHT_DRIVE_MOTOR = new CandeviceID(1);
    public final CanDeviceID BACK_LEFT_DRIVE_MOTOR = new CandeviceID();
    public final CanDeviceID BACK_RIGHTDRIVE_MOTOR = new CanDeviceID();
}
