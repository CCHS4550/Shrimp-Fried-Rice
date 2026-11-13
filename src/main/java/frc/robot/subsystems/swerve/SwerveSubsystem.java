package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Util.SubsystemDataProcessor;

import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {
  private final PIDController choreoXController = new PIDController(1.14, 0, 0);
  private final PIDController choreoYController = new PIDController(1.14, 0, 0);
  private final PIDController choreoThetaController = new PIDController(0.5, 0, 0);


  private Trajectory<SwerveSample> desiredChoreoTrajectory;
  private final Timer choreoTimer = new Timer();
  private Optional<SwerveSample> choreoSampleToBeApplied;

  private static final double CONTROLLER_DEADBAND = 0.1;
  public static final double TRANSLATION_ERROR_MARGIN_METERS_DRIVE_TO_POINT =
      Units.inchesToMeters(1.0);
  public static final double DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT = 0.02;

  public static final double ROTATION_ERROR_MARGIN_FOR_ROTATION_LOCK_DEGREES = 10.0;

  public double maxVelocityOutputForDriveToPoint = Units.feetToMeters(10.0);

  private final PIDController autoDriveToPointController = new PIDController(3.0, 0, 0.1);
  private final PIDController teleopDriveToPointController = new PIDController(3.6, 0, 0.1);

  private final SwerveRequest.SysIdSwerveTranslation translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveRotation rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();
  private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();

  private final SwerveRequest.FieldCentricFacingAngle driveAtAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  private static final double SKEW_COMPENSATION_SCALAR = -0.03;

  public enum WantedState {
    TELEOP_DRIVE,
    IDLE
  }

  public enum SystemState {
    TELEOP_DRIVE,
    IDLE
  }

  private SystemState systemState = SystemState.TELEOP_DRIVE;
  private WantedState wantedState = WantedState.TELEOP_DRIVE;

  @AutoLogOutput private Rotation2d desiredRotationForRotationLockState;

  @AutoLogOutput private Pose2d desiredPoseForDriveToPoint = new Pose2d();

  final SwerveIOInputsAutoLogged swerveInputs = new SwerveIOInputsAutoLogged();
  ModuleIOInputsAutoLogged frontLeftInputs = new ModuleIOInputsAutoLogged();
  ModuleIOInputsAutoLogged frontRightInputs = new ModuleIOInputsAutoLogged();
  ModuleIOInputsAutoLogged backLeftInputs = new ModuleIOInputsAutoLogged();
  ModuleIOInputsAutoLogged backRightInputs = new ModuleIOInputsAutoLogged();

  private final Object moduleIOLock = new Object();

  private SwerveIO io = new SwerveIO() {};
  private final CommandXboxController controller;

  private final double maxVelocity;
  private final double maxAngularVelocity;

  private double teleopVelocityCoefficient = 1.4;
  private double rotationVelocityCoefficient = 1.4;
  private double maximumAngularVelocityForDriveToPoint = Double.NaN;

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   */

  public SwerveSubsystem(
      SwerveIO io,
      CommandXboxController controller,
      double maxVelocity,
      double maxAngularVelocity) {
    this.io = io;
    this.controller = controller;
    this.maxVelocity = maxVelocity;
    this.maxAngularVelocity = maxAngularVelocity;

    // io.registerTelemetryFunction(swerveInputs);
    driveAtAngle.HeadingController = new PhoenixPIDController(5, 0, 0);

    driveAtAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

  
  }

  /**
   * Runs the quasi-static SysId test in the given direction for the given mechanism.
   *
   * @param mechanism The mechanism to characterize
   * @param direction Direction of the quasi-static SysId test
   * @return Command to run
   */

  /**
   * Runs the dynamic SysId test in the given direction for the given mechanism.
   *
   * @param mechanism The mechanism to characterize
   * @param direction Direction of the dynamic SysId test
   * @return Command to run
   */

  @Override
  public void periodic() {
    io.updateSwerveInputs(swerveInputs);
    synchronized (moduleIOLock) {
      Logger.processInputs("Subsystems/Drive", swerveInputs);
      Logger.processInputs("Subsystems/Drive/Module Data/Front Left", frontLeftInputs);
      Logger.processInputs("Subsystems/Drive/Module Data/Front Right", frontRightInputs);
      Logger.processInputs("Subsystems/Drive/Module Data/Back Left", backLeftInputs);
      Logger.processInputs("Subsystems/Drive/Module Data/Back Right", backRightInputs);
    }

    setSpeedModes();
    systemState = handleStateTransition();

    Logger.recordOutput("Subsystems/Drive/SystemState", systemState);
    Logger.recordOutput("Subsystems/Drive/DesiredState", wantedState);
    applyStates();
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case TELEOP_DRIVE -> SystemState.TELEOP_DRIVE;
      default -> SystemState.IDLE;
    };
  }

  public Rotation2d getSwerveRotation() {
    return swerveInputs.Pose.getRotation();
  }

  public SystemState getSystemState() {
    return systemState;
  }

  private void applyStates() {
    switch (systemState) { // case something something the very basic python things -- shirly 2026
        // (i was held hostage mb)
      default:
      case TELEOP_DRIVE:
        io.setSwerveState(
            new SwerveRequest.ApplyFieldSpeeds()
                .withSpeeds(calculateSpeedsBasedOnJoystickInputs())
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
        break;
    }
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return swerveInputs.ModulePositions;
  }

  private void setSpeedModes() {
    // Speed mode selection:
    //   Left bumper held → slow mode
    //   Right bumper held  → fast mode
    //   Neither           → normal mode
    // If both are somehow held simultaneously, slow mode wins.

    if (controller.leftBumper().getAsBoolean()) {
      teleopVelocityCoefficient = 0.3;
      rotationVelocityCoefficient = 0.3;
    } else if (controller.rightBumper().getAsBoolean()) {
      teleopVelocityCoefficient = 1.0;
      rotationVelocityCoefficient = 1.0;
    } else {
      teleopVelocityCoefficient = 0.6;
      rotationVelocityCoefficient = 0.6;
    }
  }

  public void setDesiredPoseForDriveToPoint(Pose2d pose) {
    this.desiredPoseForDriveToPoint = pose;
    this.maxVelocityOutputForDriveToPoint = Units.feetToMeters(10.0);
    this.maximumAngularVelocityForDriveToPoint = Double.NaN;
  }

  public void setDesiredPoseForDriveToPointWithMaximumAngularVelocity(
      Pose2d pose, double maximumAngularVelocityForDriveToPoint) {
    this.desiredPoseForDriveToPoint = pose;
    this.maxVelocityOutputForDriveToPoint = Units.feetToMeters(10.0);
    this.maximumAngularVelocityForDriveToPoint = maximumAngularVelocityForDriveToPoint;
  }

  public void setDesiredPoseForDriveToPoint(Pose2d pose, double maxVelocityOutputForDriveToPoint) {
    this.desiredPoseForDriveToPoint = pose;
    this.maxVelocityOutputForDriveToPoint = maxVelocityOutputForDriveToPoint;
  }

  public void setDesiredPoseForDriveToPointWithConstraints(
      Pose2d pose,
      double maxVelocityOutputForDriveToPoint,
      double maximumAngularVelocityForDriveToPoint) {
    this.desiredPoseForDriveToPoint = pose;
    this.maxVelocityOutputForDriveToPoint = maxVelocityOutputForDriveToPoint;
    this.maximumAngularVelocityForDriveToPoint = maximumAngularVelocityForDriveToPoint;
  }

  private ChassisSpeeds calculateSpeedsBasedOnJoystickInputs() {
    if (DriverStation.getAlliance().isEmpty()) {
      return new ChassisSpeeds(0, 0, 0);
    }

    double flipControls = 1;

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      flipControls = -1;
    }
    double xMagnitude =
        flipControls * MathUtil.applyDeadband(controller.getLeftY(), CONTROLLER_DEADBAND);
    double yMagnitude =
        flipControls * MathUtil.applyDeadband(controller.getLeftX(), CONTROLLER_DEADBAND);
    double angularMagnitude =
        -1 * MathUtil.applyDeadband(controller.getRightX(), CONTROLLER_DEADBAND);

    xMagnitude = Math.copySign(xMagnitude * xMagnitude, xMagnitude);
    yMagnitude = Math.copySign(yMagnitude * yMagnitude, yMagnitude);
    angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);

    double xVelocity =
        xMagnitude * maxVelocity * teleopVelocityCoefficient;

    double yVelocity =
        yMagnitude * maxVelocity  * teleopVelocityCoefficient;

    double angularVelocity = angularMagnitude * maxAngularVelocity * rotationVelocityCoefficient;

    Rotation2d skewCompensationFactor =
        Rotation2d.fromRadians(
            swerveInputs.Speeds.omegaRadiansPerSecond * SKEW_COMPENSATION_SCALAR);

    return ChassisSpeeds.fromRobotRelativeSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(xVelocity, yVelocity, -angularVelocity),
            swerveInputs.Pose.getRotation()),
        swerveInputs.Pose.getRotation().plus(skewCompensationFactor));
  }

  public void resetTranslationAndRotation(Pose2d pose2d) {
    resetTranslation(pose2d);
    resetRotation(pose2d.getRotation());
  }

  public void resetRotation(Rotation2d rotation2d) {
    io.resetToParamaterizedRotation(rotation2d);
  }

  public void resetTranslation(Pose2d pose) {
    io.resetRobotTranslation(pose.getTranslation());
  }

  public void resetRotationBasedOnAlliance() {
    io.resetRotation();
  }

  public void setWantedState(WantedState state) {
    this.wantedState = state;
  }

  public boolean isAtDesiredRotation() {
    return isAtDesiredRotation(Units.degreesToRadians(0.5));
  }

  public boolean isAtDesiredRotation(double tolerance) {
    return driveAtAngle.HeadingController.getPositionError() < tolerance;
  }

  public void setTeleopVelocityCoefficient(double teleopVelocityCoefficient) {
    this.teleopVelocityCoefficient = teleopVelocityCoefficient;
  }

  public void setRotationVelocityCoefficient(double rotationVelocityCoefficient) {
    this.rotationVelocityCoefficient = rotationVelocityCoefficient;
  }


  /** Adds a new timestamped vision measurement. */

  // @Override
  // public void accept(Pose2d pose, double time, Matrix<N3, N1> StdDevs) {
  //   this.resetTranslationAndRotation(pose);
  // }
}