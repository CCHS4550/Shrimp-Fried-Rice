package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;

public class DriveIOHardware implements DriveIO {
    private final TalonFX frontLeftMotor = new TalonFX(DriveConstants.FLdeviceID);
    private final TalonFX frontRightMotor = new TalonFX(DriveConstants.FRdeviceID);
    private final TalonFX backLeftMotor = new TalonFX(DriveConstants.BLdeviceID);
    private final TalonFX backRightMotor = new TalonFX(DriveConstants.BRdeviceID);

    private final DifferentialDrive differentialDrive;

    private final StatusSignal<Angle> leftPositionSignal;
    private final StatusSignal<AngularVelocity> leftVelocitySignal;
    private final StatusSignal<Voltage> leftVoltageSignal;
    private final StatusSignal<Current> leftSupplyCurrentSignal;
    private final StatusSignal<Current> leftStatorCurrentSignal;

    private final StatusSignal<Angle> rightPositionSignal;
    private final StatusSignal<AngularVelocity> rightVelocitySignal;
    private final StatusSignal<Voltage> rightVoltageSignal;
    private final StatusSignal<Current> rightSupplyCurrentSignal;
    private final StatusSignal<Current> rightStatorCurrentSignal;

    public DriveIOHardware() {
        configureMotor(frontLeftMotor, InvertedValue.Clockwise_Positive);
        configureMotor(backLeftMotor, InvertedValue.Clockwise_Positive);
        configureMotor(frontRightMotor, InvertedValue.CounterClockwise_Positive);
        configureMotor(backRightMotor, InvertedValue.CounterClockwise_Positive);

        Follower leftFollower = new Follower(DriveConstants.FLdeviceID, MotorAlignmentValue.Aligned);
        Follower rightFollower = new Follower(DriveConstants.FRdeviceID, MotorAlignmentValue.Aligned);
        backLeftMotor.setControl(leftFollower);
        backRightMotor.setControl(rightFollower);

        frontLeftMotor.setPosition(0.0);
        frontRightMotor.setPosition(0.0);

        leftPositionSignal = frontLeftMotor.getPosition();
        leftVelocitySignal = frontLeftMotor.getVelocity();
        leftVoltageSignal = frontLeftMotor.getMotorVoltage();
        leftSupplyCurrentSignal = frontLeftMotor.getSupplyCurrent();
        leftStatorCurrentSignal = frontLeftMotor.getStatorCurrent();

        rightPositionSignal = frontRightMotor.getPosition();
        rightVelocitySignal = frontRightMotor.getVelocity();
        rightVoltageSignal = frontRightMotor.getMotorVoltage();
        rightSupplyCurrentSignal = frontRightMotor.getSupplyCurrent();
        rightStatorCurrentSignal = frontRightMotor.getStatorCurrent();

        differentialDrive = new DifferentialDrive(frontLeftMotor::set, frontRightMotor::set);
    }

    private void configureMotor(TalonFX motor, InvertedValue inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        MotorOutputConfigs motorOutput = config.MotorOutput;
        motorOutput.NeutralMode = NeutralModeValue.Brake;
        motorOutput.Inverted = inverted;

        CurrentLimitsConfigs currentLimits = config.CurrentLimits;
        currentLimits.SupplyCurrentLimitEnable = true;
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = DriveConstants.supplyCurrentLimit;
        currentLimits.StatorCurrentLimit = DriveConstants.statorCurrentLimit;

        TalonFXConfigurator configurator = motor.getConfigurator();
        configurator.apply(config);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            leftPositionSignal,
            leftVelocitySignal,
            leftVoltageSignal,
            leftSupplyCurrentSignal,
            leftStatorCurrentSignal,
            rightPositionSignal,
            rightVelocitySignal,
            rightVoltageSignal,
            rightSupplyCurrentSignal,
            rightStatorCurrentSignal
        );

        inputs.leftPosition = leftPositionSignal.getValueAsDouble();
        inputs.leftVelocity = leftVelocitySignal.getValueAsDouble();
        inputs.leftAppliedVolts = leftVoltageSignal.getValueAsDouble();
        inputs.leftSupplyCurrentAmps = leftSupplyCurrentSignal.getValueAsDouble();
        inputs.leftStatorCurrentAmps = leftStatorCurrentSignal.getValueAsDouble();

        inputs.rightPosition = rightPositionSignal.getValueAsDouble();
        inputs.rightVelocity = rightVelocitySignal.getValueAsDouble();
        inputs.rightAppliedVolts = rightVoltageSignal.getValueAsDouble();
        inputs.rightSupplyCurrentAmps = rightSupplyCurrentSignal.getValueAsDouble();
        inputs.rightStatorCurrentAmps = rightStatorCurrentSignal.getValueAsDouble();
    }

    @Override
    public void stopRobot() {
        differentialDrive.stopMotor();
        frontLeftMotor.stopMotor();
        frontRightMotor.stopMotor();
    }

    @Override
    public void differentialDrive(double forwardSpeed, double turnSpeed) {
        differentialDrive.arcadeDrive(forwardSpeed, turnSpeed);
    }

    @Override
    public void tankDrive(double leftSpeed, double rightSpeed) {
        differentialDrive.tankDrive(leftSpeed, rightSpeed);
    }

    @Override
    public void curvatureDrive(double forwardSpeed, double turnSpeed, boolean allowTurnInPlace) {
        differentialDrive.curvatureDrive(forwardSpeed, turnSpeed, allowTurnInPlace);
    }
}
