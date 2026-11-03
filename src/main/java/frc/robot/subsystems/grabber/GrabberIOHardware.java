package frc.robot.subsystems.grabber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.GrabberConstants;

public class GrabberIOHardware implements GrabberIO {
    private final TalonFX grabberMotor;
    private final TalonFXConfiguration grabberConfig;

    private final StatusSignal<Voltage> grabberAppliedVolts;
    private final StatusSignal<Angle> grabberPositionRot;
    private final StatusSignal<Current> grabberSupplyCurrentAmps;
    private final StatusSignal<Current> grabberStatorCurrentAmps;
    private final StatusSignal<AngularVelocity> grabberVelocityRotPerSec;
    private final StatusSignal<AngularAcceleration> grabberAccelRotPerSecSquared;
    private final StatusSignal<Temperature> grabberMotorTemp;

    public GrabberIOHardware() {
        grabberMotor = new TalonFX(GrabberConstants.motorDeviceID);
        grabberConfig = new TalonFXConfiguration();

        CurrentLimitsConfigs currentLimits = grabberConfig.CurrentLimits;
        currentLimits.SupplyCurrentLimitEnable = true;
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = GrabberConstants.supplyCurrentLimit;
        currentLimits.StatorCurrentLimit = GrabberConstants.statorCurrentLimit;

        MotorOutputConfigs motorOutput = grabberConfig.MotorOutput;
        motorOutput.NeutralMode = NeutralModeValue.Brake;
        motorOutput.Inverted = InvertedValue.Clockwise_Positive;

        TalonFXConfigurator configurator = grabberMotor.getConfigurator();
        configurator.apply(grabberConfig);

        grabberMotor.setPosition(0.0);
        grabberAppliedVolts = grabberMotor.getMotorVoltage();
        grabberPositionRot = grabberMotor.getPosition();
        grabberSupplyCurrentAmps = grabberMotor.getSupplyCurrent();
        grabberStatorCurrentAmps = grabberMotor.getStatorCurrent();
        grabberVelocityRotPerSec = grabberMotor.getVelocity();
        grabberAccelRotPerSecSquared = grabberMotor.getAcceleration();
        grabberMotorTemp = grabberMotor.getDeviceTemp();
    }

    @Override
    public void updateInputs(GrabberIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            grabberAppliedVolts,
            grabberPositionRot,
            grabberSupplyCurrentAmps,
            grabberStatorCurrentAmps,
            grabberVelocityRotPerSec,
            grabberAccelRotPerSecSquared,
            grabberMotorTemp
        );

        double positionRotations = grabberPositionRot.getValueAsDouble();
        double velocityRotationsPerSec = grabberVelocityRotPerSec.getValueAsDouble();
        double accelRotationsPerSecSquared = grabberAccelRotPerSecSquared.getValueAsDouble();

        inputs.grabberVoltage = grabberAppliedVolts.getValueAsDouble();
        inputs.grabberPositionRad = Units.rotationsToRadians(positionRotations);
        inputs.grabberSupplyCurrent = grabberSupplyCurrentAmps.getValueAsDouble();
        inputs.grabberStatorCurrent = grabberStatorCurrentAmps.getValueAsDouble();
        inputs.grabberTemperature = grabberMotorTemp.getValueAsDouble();
        inputs.grabberVelocityRadPerSec = Units.rotationsToRadians(velocityRotationsPerSec);
        inputs.grabberAccelRadPerSecSquared = Units.rotationsToRadians(accelRotationsPerSecSquared);
    }

    @Override
    public void setVoltage(double voltage) {
        grabberMotor.setVoltage(voltage);
    }

    @Override
    public void setSpeed(double speed) {
        grabberMotor.set(speed);
    }

    @Override
    public void stop() {
        grabberMotor.stopMotor();
    }
}
