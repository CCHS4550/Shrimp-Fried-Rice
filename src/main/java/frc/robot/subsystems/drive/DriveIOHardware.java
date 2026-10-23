package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.SparkMaxMotorController;

public class DriveIOHardware implements DriveIO {
    private final SparkMaxMotorController frontLeftMotor = new SparkMaxMotorController(
        "FrontLeftMotor",
        Constants.DriveConstants.FLdeviceID,
        MotorType.kBrushless,
        true,
        IdleMode.kBrake
    );
    private final SparkMaxMotorController frontRightMotor = new SparkMaxMotorController(
        "FrontRightMotor",
        Constants.DriveConstants.FRdeviceID,
        MotorType.kBrushless,
        false,
        IdleMode.kBrake
    );
    private final SparkMaxMotorController backLeftMotor = new SparkMaxMotorController(
        "BackLeftMotor",
        Constants.DriveConstants.BLdeviceID,
        MotorType.kBrushless,
        true,
        IdleMode.kBrake
    );
    private final SparkMaxMotorController backRightMotor = new SparkMaxMotorController(
        "BackRightMotor",
        Constants.DriveConstants.BRdeviceID,
        MotorType.kBrushless,
        false,
        IdleMode.kBrake
    );

    private final DifferentialDrive differentialDrive;

    public DriveIOHardware() {
        backLeftMotor.follow(frontLeftMotor);
        backRightMotor.follow(frontRightMotor);
        differentialDrive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
    }

    @Override
    public void stopRobot() {
        differentialDrive.stopMotor();
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
