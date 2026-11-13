package frc.robot.Config;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.util.Units;
import frc.robot.Util.CanDeviceID;

public class RobotConfig {
    private static final CANBus CANIVORE_CANBUS  = new CANBus("CANivore");

    public final CanDeviceID GYRO = new CanDeviceID(13);

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

    public final CanDeviceID ELEVATION_MOTOR = new CanDeviceID(15, CANIVORE_CANBUS);
    public final CanDeviceID GRABBER_Motor = new CanDeviceID(19, CANIVORE_CANBUS);

    private final ElevatorConfig elevatorConfig;
    private final GrabberConfig grabberConfig;



  /**
   * The coupled gear ratio between the CanCoder and the drive motor. Every 1 rotation of the steer
   * motor results in coupled ratio of drive turns.
   */

  // TODO: find and fill
  private final double COUPLING_GEAR_RATIO = 0.0;

  // 3.857142857142857

  /**
   * Wheelbase length is the distance between the front and back wheels. Positive x values represent
   * moving towards the front of the robot
   */

  // TODO: find and fill
  private final double WHEELBASE_LENGTH_METERS = Units.inchesToMeters(27.25);

  /**
   * Wheel track width is the distance between the left and right wheels. Positive y values
   * represent moving towards the left of the robot.
   */

  // TODO: find and fill
  private final double WHEEL_TRACK_WIDTH_METERS = Units.inchesToMeters(27.25);

  /** The maximum speed of the robot in meters per second. */

  // TODO: find and fill
  private final double MAX_SPEED_METERS_PER_SECOND = 5;

  // CANcoder offsets of the swerve modules

  // TODO: find and fill
  private final double FRONT_LEFT_STEER_OFFSET_ROTATIONS = 0.141845703125;
  private final double FRONT_RIGHT_STEER_OFFSET_ROTATIONS = -0.37451171875;
  private final double BACK_LEFT_STEER_OFFSET_ROTATIONS = -0.310302734375;
  private final double BACK_RIGHT_STEER_OFFSET_ROTATIONS = 0.022705078125;

  // TODO: find and fill
  private final int GYRO_MOUNTING_ANGLE = 0;
  private final double GYRO_ERROR = 1.6;

  // Robot configuration
  private final SwerveDrivetrainConstants swerveDrivetrainConstants;
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
      moduleConstants;
  

  // PathPlanner config constants
  private final double ROBOT_MASS_KG = 25.338;
  private final double ROBOT_MOI = 6.7;
  private final double WHEEL_COF = 1.2;
  private RobotConfig PP_CONFIG;

  

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
    private final double ELEVATOR_GEAR_RATIO = 80;

    Pigeon2Configuration pigeon2Configuration = new Pigeon2Configuration();


    public RobotConfig(){
        grabberConfig = new GrabberConfig().
                            withGrabberKp(21).
                            withGrabberKi(0).
                            withGrabberKd(1).
                            withGrabberKs(0.0).
                            withGrabberKv(0.5);

        elevatorConfig = new ElevatorConfig().
                             withElevatorKp(21).
                             withElevatorKi(0).
                             withElevatorKd(1).
                             withElevatorKs(0.0).
                             withElevatorKv(0.5)
                             .withElevatorKg(67);

        swerveDrivetrainConstants =
        new SwerveDrivetrainConstants()
            .withCANBusName("rio")
            .withPigeon2Id(GYRO.getDeviceNumber())
            .withPigeon2Configs(pigeon2Configuration);

        moduleConstants = new SwerveModuleConstants[4];
        moduleConstants[0] =
        new SwerveModuleConstants<
                TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorId(FRONT_LEFT_DRIVE_MOTOR.getDeviceNumber())
            .withSteerMotorId(FRONT_LEFT_STEER_MOTOR.getDeviceNumber())
            .withEncoderId(FRONT_LEFT_STEER_ENCODER.getDeviceNumber())
            .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(STEER_GEAR_RATIO)
            .withCouplingGearRatio(COUPLING_GEAR_RATIO)
            .withDriveMotorInverted(true) // true
            .withSteerMotorInverted(false)
            .withEncoderInverted(false)
            .withEncoderOffset(FRONT_LEFT_STEER_OFFSET_ROTATIONS)
            .withLocationX(WHEELBASE_LENGTH_METERS / 2)
            .withLocationY(WHEEL_TRACK_WIDTH_METERS / 2)
            .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
            .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
            
           
            .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
            .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
            .withDriveMotorInitialConfigs(new TalonFXConfiguration())
            .withSteerMotorInitialConfigs(new TalonFXConfiguration())
            .withEncoderInitialConfigs(new CANcoderConfiguration())
            .withDriveFrictionVoltage(0.25)
            .withSteerFrictionVoltage(0.001)
            .withDriveInertia(0.001)
            .withSteerInertia(0.00001)
            .withSlipCurrent(120) // TODO MEASURE
            .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
            .withSpeedAt12Volts(90)
            .withWheelRadius(WHEEL_RADIUS_METERS);

        moduleConstants[1] =
            new SwerveModuleConstants<
                TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorId(FRONT_RIGHT_DRIVE_MOTOR.getDeviceNumber())
            .withSteerMotorId(FRONT_RIGHT_STEER_MOTOR.getDeviceNumber())
            .withEncoderId(FRONT_RIGHT_STEER_ENCODER.getDeviceNumber())
            .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(STEER_GEAR_RATIO)
            .withCouplingGearRatio(COUPLING_GEAR_RATIO)
            .withDriveMotorInverted(false)
            .withSteerMotorInverted(false)
            .withEncoderInverted(false)
            .withEncoderOffset(FRONT_RIGHT_STEER_OFFSET_ROTATIONS)
            .withLocationX(WHEELBASE_LENGTH_METERS / 2)
            .withLocationY(-WHEEL_TRACK_WIDTH_METERS / 2)
            .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
            .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)

            .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
            .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
            .withDriveMotorInitialConfigs(new TalonFXConfiguration())
            .withSteerMotorInitialConfigs(new TalonFXConfiguration())
            .withEncoderInitialConfigs(new CANcoderConfiguration())
            .withDriveFrictionVoltage(0.25)
            .withSteerFrictionVoltage(0.001)
            .withDriveInertia(0.001)
            .withSteerInertia(0.00001)
            .withSlipCurrent(120) // TODO MEASURE
            .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
            .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
            .withWheelRadius(WHEEL_RADIUS_METERS);

        moduleConstants[2] =
            new SwerveModuleConstants<
                TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorId(BACK_LEFT_DRIVE_MOTOR.getDeviceNumber())
            .withSteerMotorId(BACK_LEFT_STEER_MOTOR.getDeviceNumber())
            .withEncoderId(BACK_LEFT_STEER_ENCODER.getDeviceNumber())
            .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(STEER_GEAR_RATIO)
            .withCouplingGearRatio(COUPLING_GEAR_RATIO)
            .withDriveMotorInverted(true) // true
            .withSteerMotorInverted(false)
            .withEncoderInverted(false)
            .withEncoderOffset(BACK_LEFT_STEER_OFFSET_ROTATIONS)
            .withLocationX(-WHEELBASE_LENGTH_METERS / 2)
            .withLocationY(WHEEL_TRACK_WIDTH_METERS / 2)
            .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
            .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)

            .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
            .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
            .withDriveMotorInitialConfigs(new TalonFXConfiguration())
            .withSteerMotorInitialConfigs(new TalonFXConfiguration())
            .withEncoderInitialConfigs(new CANcoderConfiguration())
            .withDriveFrictionVoltage(0.25)
            .withSteerFrictionVoltage(0.001)
            .withDriveInertia(0.001)
            .withSteerInertia(0.00001)
            .withSlipCurrent(120) // TODO MEASURE
            .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
            .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
            .withWheelRadius(WHEEL_RADIUS_METERS);

        moduleConstants[3] =
            new SwerveModuleConstants<
                TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorId(BACK_RIGHT_DRIVE_MOTOR.getDeviceNumber())
            .withSteerMotorId(BACK_RIGHT_STEER_MOTOR.getDeviceNumber())
            .withEncoderId(BACK_RIGHT_STEER_ENCODER.getDeviceNumber())
            .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(STEER_GEAR_RATIO)
            .withCouplingGearRatio(COUPLING_GEAR_RATIO)
            .withDriveMotorInverted(false)
            .withSteerMotorInverted(false)
            .withEncoderInverted(false)
            .withEncoderOffset(BACK_RIGHT_STEER_OFFSET_ROTATIONS)
            .withLocationX(-WHEELBASE_LENGTH_METERS / 2)
            .withLocationY(-WHEEL_TRACK_WIDTH_METERS / 2)
            .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
            .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)

            .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
            .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
            .withDriveMotorInitialConfigs(new TalonFXConfiguration())
            .withSteerMotorInitialConfigs(new TalonFXConfiguration())
            .withEncoderInitialConfigs(new CANcoderConfiguration())
            .withDriveFrictionVoltage(0.25)
            .withSteerFrictionVoltage(0.001)
            .withDriveInertia(0.001)
            .withSteerInertia(0.00001)
            .withSlipCurrent(120) // TODO MEASURE
            .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
            .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
            .withWheelRadius(WHEEL_RADIUS_METERS);

    }

    public GrabberConfig getGrabberConfig(){
        return grabberConfig;

    }
    public ElevatorConfig getElevatorConfig(){
        return elevatorConfig;
    }

    public SwerveDrivetrainConstants getSwerveDrivetrainConstants(){
        return swerveDrivetrainConstants;
    }

    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
      getModuleConstants() {
    return moduleConstants;
  }
}
