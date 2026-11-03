package frc.robot.subsystems.elevators;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOHardware implements ElevatorIO {
    private final TalonFX motor = new TalonFX(ElevatorConstants.motorDeviceID);
    private final PositionVoltage positionRequest = new PositionVoltage(0.0);

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> statorCurrentSignal;

    public ElevatorIOHardware() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        MotorOutputConfigs motorOutput = config.MotorOutput;
        motorOutput.NeutralMode = NeutralModeValue.Brake;
        motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        CurrentLimitsConfigs currentLimits = config.CurrentLimits;
        currentLimits.SupplyCurrentLimitEnable = true;
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = ElevatorConstants.supplyCurrentLimit;
        currentLimits.StatorCurrentLimit = ElevatorConstants.statorCurrentLimit;

        Slot0Configs slot0 = config.Slot0;
        slot0.kP = ElevatorConstants.kP;
        slot0.kI = ElevatorConstants.kI;
        slot0.kD = ElevatorConstants.kD;
        slot0.kG = ElevatorConstants.kG;
        slot0.GravityType = GravityTypeValue.Elevator_Static;

        TalonFXConfigurator configurator = motor.getConfigurator();
        configurator.apply(config);

        motor.setPosition(0.0);

        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        voltageSignal = motor.getMotorVoltage();
        supplyCurrentSignal = motor.getSupplyCurrent();
        statorCurrentSignal = motor.getStatorCurrent();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            positionSignal,
            velocitySignal,
            voltageSignal,
            supplyCurrentSignal,
            statorCurrentSignal
        );
        inputs.position = positionSignal.getValueAsDouble();
        inputs.velocity = velocitySignal.getValueAsDouble();
        inputs.appliedVolts = voltageSignal.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrentSignal.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrentSignal.getValueAsDouble();
    }

    @Override
    public void stopElevator() {
        motor.stopMotor();
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setPosition(double position) {
        PositionVoltage request = positionRequest.withPosition(position);
        motor.setControl(request);
    }

    @Override
    public double getPosition() {
        positionSignal.refresh();
        return positionSignal.getValueAsDouble();
    }

    @Override
    public double getVelocity() {
        velocitySignal.refresh();
        return velocitySignal.getValueAsDouble();
    }

    @Override
    public void resetEncoder() {
        motor.setPosition(0.0);
    }
}

