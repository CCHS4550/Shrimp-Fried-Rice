package frc.robot.subsystems.Drive.module;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public class ModuleIOSpark implements ModuleIO{
    private final int module;

    private final SparkMax driveMotor;
    private final SparkMax turnMotor;

    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;

    public ModuleIOSpark(int module){
        this.module = module;

        int id;
        boolean inverted;

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        SparkMaxConfig turnConfig = new SparkMaxConfig();
        
        //1: FL, 2: FR, 3: BL, 4: BR
        switch(this.module)
        {
            default: 
                case 1:
                    id = Constants.DriveConstants.frontLeftDriveCanId;
                    inverted = Constants.DriveConstants.frontLeftDriveInverted;
                break;

            case 2:
                id = Constants.DriveConstants.frontRightDriveCanId;
                inverted = Constants.DriveConstants.frontRightDriveInverted;
                break;

            case 3:
                id = Constants.DriveConstants.backLeftDriveCanId;
                inverted = Constants.DriveConstants.backLeftDriveInverted;
                break;

            case 4:
                id = Constants.DriveConstants.backRightDriveCanId;
                inverted = Constants.DriveConstants.backRightDriveInverted;
                break;
        }

        driveMotor = new SparkMax(id, MotorType.kBrushless);

        driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.DriveConstants.driveMotorCurrentLimit)
        .voltageCompensation(12)
        .encoder
        .positionConversionFactor(Constants.DriveConstants.driveEncoderPositionFactor)
        .velocityConversionFactor(Constants.DriveConstants.driveEncoderVelocityFactor);

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        switch(module)
        {
            default: 
                case 1:
                    id = Constants.DriveConstants.frontLeftTurnCanId;
                    inverted = Constants.DriveConstants.frontLeftTurnInverted;
                break;

            case 2:
                id = Constants.DriveConstants.frontRightTurnCanId;
                inverted = Constants.DriveConstants.frontRightTurnInverted;
                break;

            case 3:
                id = Constants.DriveConstants.backLeftTurnCanId;
                inverted = Constants.DriveConstants.backLeftTurnInverted;
                break;

            case 4:
                id = Constants.DriveConstants.backRightTurnCanId;
                inverted = Constants.DriveConstants.backRightTurnInverted;
                break;
        }

        turnMotor = new SparkMax(id, MotorType.kBrushless);

        turnConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.DriveConstants.turnMotorCurrentLimit)
        .voltageCompensation(12)
        .encoder
        .positionConversionFactor(Constants.DriveConstants.turnEncoderPositionFactor)
        .velocityConversionFactor(Constants.DriveConstants.turnEncoderVelocityFactor);

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveController = driveMotor.getClosedLoopController();
        turnController = turnMotor.getClosedLoopController();


    }


    public void updateModuleInputs(SparkMax driveMotor ){
        
        driveMotor.getVelocity();

    }


    @Override
    public void setDriveVelocity(double velocity){
        driveMotor.setVoltage(velocity);
    }

    @Override
    public void setTurnPosition(double velocity){
        turnMotor.setVoltage(velocity);
    }

}

