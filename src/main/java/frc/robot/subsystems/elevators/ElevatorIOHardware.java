package frc.robot.subsystems.elevators;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Config.RobotConfig;


public class ElevatorIOHardware implements ElevatorIO {
    private final SparkMax elevatorSpark;
    private final RelativeEncoder elevatorEncoder;
    private final SparkClosedLoopController closedLoopController;
    private final SparkMaxConfig elevatorConfig;
    private final Debouncer elevatorDebouncer = new Debouncer(0.5);

    public ElevatorIOHardware(RobotConfig robotConfig) {
        elevatorSpark = new SparkMax(robotConfig.ELEVATION_MOTOR.getDeviceNumber(), MotorType.kBrushless);
        elevatorEncoder = elevatorSpark.getEncoder();
        closedLoopController = elevatorSpark.getClosedLoopController();
        elevatorConfig = new SparkMaxConfig();

        elevatorConfig.inverted(false);
        elevatorConfig.idleMode(IdleMode.kBrake);
        elevatorConfig.smartCurrentLimit(40);
        elevatorConfig.voltageCompensation(12.0);

        EncoderConfig encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(1.0);
        encoderConfig.velocityConversionFactor(1.0 / 60.0);
        encoderConfig.uvwMeasurementPeriod(20);
        encoderConfig.uvwAverageDepth(2);
        elevatorConfig.apply(encoderConfig);

        FeedForwardConfig feedForwardConfig = new FeedForwardConfig();
        feedForwardConfig.kG(robotConfig.getElevatorConfig().elevatorKg);

        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig.pid(robotConfig.getElevatorConfig().elevatorKp, robotConfig.getElevatorConfig().elevatorKi, robotConfig.getElevatorConfig().elevatorKd);
        closedLoopConfig.outputRange(-1.0, 1.0);
        closedLoopConfig.apply(feedForwardConfig);
        elevatorConfig.apply(closedLoopConfig);

        SignalsConfig signalsConfig = new SignalsConfig();
        signalsConfig.primaryEncoderPositionAlwaysOn(true);
        signalsConfig.primaryEncoderVelocityAlwaysOn(true);
        signalsConfig.primaryEncoderPositionPeriodMs(20);
        signalsConfig.primaryEncoderVelocityPeriodMs(20);
        signalsConfig.appliedOutputPeriodMs(20);
        signalsConfig.busVoltagePeriodMs(20);
        signalsConfig.outputCurrentPeriodMs(20);
        elevatorConfig.apply(signalsConfig);

        elevatorSpark.configure(
            elevatorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        elevatorEncoder.setPosition(0.0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        REVLibError lastError = elevatorSpark.getLastError();
        boolean isOk = lastError == REVLibError.kOk;
        inputs.connected = elevatorDebouncer.calculate(isOk);

        if (inputs.connected) {
            double appliedOutput = elevatorSpark.getAppliedOutput();
            double busVoltage = elevatorSpark.getBusVoltage();

            inputs.position = elevatorEncoder.getPosition();
            inputs.velocity = elevatorEncoder.getVelocity();
            inputs.appliedVolts = appliedOutput * busVoltage;
            inputs.currentAmps = elevatorSpark.getOutputCurrent();
        }
    }

    @Override
    public void stopElevator() {
        elevatorSpark.stopMotor();
    }

    @Override
    public void setSpeed(double speed) {
        double clampedSpeed = MathUtil.clamp(speed, -1.0, 1.0);
        elevatorSpark.set(clampedSpeed);
    }

    @Override
    public void setVoltage(double volts) {
        double clampedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        elevatorSpark.setVoltage(clampedVolts);
    }

    @Override
    public void setPosition(double position) {
        closedLoopController.setSetpoint(position, ControlType.kPosition);
    }

    @Override
    public double getPosition() {
        return elevatorEncoder.getPosition();
    }

    @Override
    public double getVelocity() {
        return elevatorEncoder.getVelocity();
    }

    @Override
    public void resetEncoder() {
        elevatorEncoder.setPosition(0.0);
    }
}