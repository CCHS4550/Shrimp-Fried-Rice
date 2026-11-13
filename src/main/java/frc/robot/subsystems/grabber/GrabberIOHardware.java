package frc.robot.subsystems.grabber;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import frc.robot.Config.RobotConfig;

public class GrabberIOHardware implements GrabberIO {
    private final SparkMax grabberSpark;
    private final RelativeEncoder grabberEncoder;
    private final SparkMaxConfig grabberConfig;
    private final Debouncer grabberDebouncer = new Debouncer(0.5);

    public GrabberIOHardware(RobotConfig config) {
        grabberSpark = new SparkMax(config.GRABBER_Motor.getDeviceNumber(), MotorType.kBrushless);
        grabberEncoder = grabberSpark.getEncoder();
        grabberConfig = new SparkMaxConfig();

        grabberConfig.inverted(false);
        grabberConfig.idleMode(IdleMode.kBrake);
        grabberConfig.smartCurrentLimit(40);
        grabberConfig.voltageCompensation(12.0);

        EncoderConfig encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(Units.rotationsToRadians(1.0));
        encoderConfig.velocityConversionFactor(Units.rotationsPerMinuteToRadiansPerSecond(1.0));
        encoderConfig.uvwMeasurementPeriod(20);
        encoderConfig.uvwAverageDepth(2);
        grabberConfig.apply(encoderConfig);

        SignalsConfig signalsConfig = new SignalsConfig();
        signalsConfig.primaryEncoderPositionAlwaysOn(true);
        signalsConfig.primaryEncoderVelocityAlwaysOn(true);
        signalsConfig.primaryEncoderVelocityPeriodMs(20);
        signalsConfig.appliedOutputPeriodMs(20);
        signalsConfig.busVoltagePeriodMs(20);
        signalsConfig.outputCurrentPeriodMs(20);
        grabberConfig.apply(signalsConfig);

        grabberSpark.configure(
            grabberConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        grabberEncoder.setPosition(0.0);
    }

    @Override
    public void updateInputs(GrabberIOInputs inputs) {
        REVLibError lastError = grabberSpark.getLastError();
        boolean isOk = lastError == REVLibError.kOk;
        inputs.grabberConnected = grabberDebouncer.calculate(isOk);

        if (inputs.grabberConnected) {
            double appliedOutput = grabberSpark.getAppliedOutput();
            double busVoltage = grabberSpark.getBusVoltage();

            inputs.grabberPositionRad = grabberEncoder.getPosition();
            inputs.grabberVelocityRadPerSec = grabberEncoder.getVelocity();
            inputs.grabberAppliedVolts = appliedOutput * busVoltage;
            inputs.grabberCurrentAmps = grabberSpark.getOutputCurrent();
        }
    }

    @Override
    public void setVoltage(double volts) {
        double clampedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        grabberSpark.setVoltage(clampedVolts);
    }

    @Override
    public void setSpeed(double speed) {
        double clampedSpeed = MathUtil.clamp(speed, -1.0, 1.0);
        grabberSpark.set(clampedSpeed);
    }

    @Override
    public void stop() {
        grabberSpark.stopMotor();
    }

    @Override
    public void resetEncoder() {
        grabberEncoder.setPosition(0.0);
    }
}