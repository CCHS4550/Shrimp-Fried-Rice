package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive.module.ModuleIO;

public class Swerve extends SubsystemBase {

    //4 moduleIO instantiations
    
    //private ModuleIO [] io;

    private final ModuleIO frontRight;
    private final ModuleIO frontLeft;
    private final ModuleIO backRight;
    private final ModuleIO backLeft;

    private final ModuleIOInputsAutoLogged frontRightInputs = new ModuleIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged frontLeftInputs = new ModuleIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged backRightInputs = new ModuleIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged backLeftInputs = new ModuleIOInputsAutoLogged();



    private final ModuleIOInputsAutoLogged[] swerveModuleInputs;

    private SwerveModuleState[] desiredModuleStates;

    private ChassisSpeeds chassisSpeeds;

    private SwerveDriveKinematics kinematics;

    private CommandXboxController controller;

    public static final double CONTROLLER_DEADBAND = 0.1;

    public double teleopVelocityCoefficient = 1.4;
    private double rotationVelocityCoefficient = 1.4;

    private final double maxVelocity;
    private final double maxAngularVelocity;

    private static final double SKEW_COMPENSATION_SCALAR = 0.03;

    public enum WantedState
    {
        TELEOP_DRIVE,
        IDLE
    }

    public enum SystemState
    {
        TELEOP_DRIVE,
        IDLE
    }
    
    private SystemState systemState = SystemState.TELEOP_DRIVE;
    private WantedState wantedState = WantedState.TELEOP_DRIVE;


    public Swerve (ModuleIO frontLeft, ModuleIO frontRight, ModuleIO backLeft, ModuleIO backRight, CommandXboxController controller){
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        this.controller = controller;
    }

    @Override
    public void periodic(){

        frontRight.updateInputs(frontRightInputs);
        frontLeft.updateInputs(frontLeftInputs);
        backRight.updateInputs(backRightInputs);
        backLeft.updateInputs(backLeftInputs);

     
        Logger.processInputs("Subsystems/Drive/Module Data/Front Right", frontRightInputs);
        Logger.processInputs("Subsystems/Drive/Module Data/Front Left", frontLeftInputs);
        Logger.processInputs("Subsystems/Drive/Module Data/Back Left", backRightInputs);
        Logger.processInputs("Subsystems/Drive/Module Data/Back Left", backLeftInputs);

        systemState = handleStateTransition();
        applayState();
        updateModuleInputs();

    }

    public SystemState getSystemState(){
        return systemState;
    }
    

    public ChassisSpeeds calculateSpeedsBasedOnJoystickInputs(){
        double flipControls = 1;

        double xMagnitude =
            flipControls * MathUtil.applyDeadband(controller.getLeftY(),CONTROLLER_DEADBAND);

        double yMagnitude =
            flipControls * MathUtil.applyDeadband(controller.getLeftX(),CONTROLLER_DEADBAND);
        
        double angularMagnitude =
            -1 * MathUtil.applyDeadband(controller.getLeftY(),CONTROLLER_DEADBAND);

        xMagnitude = Math.copySign(xMagnitude * xMagnitude, xMagnitude);
        yMagnitude = Math.copySign(yMagnitude * yMagnitude, yMagnitude);
        angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);


        // need to test out for x, y
        double xVelocity = 
            -xMagnitude * maxVelocity * teleopVelocityCoefficient;

        double yVelocity = 
            -yMagnitude * maxVelocity  * teleopVelocityCoefficient;

        double angularVelocity = 
            angularMagnitude * maxAngularVelocity * rotationVelocityCoefficient;

        Rotation2d skewCompensationFactor = 
            Rotation2d.fromRadians(
                .omegaRadiansPerSecond * SKEW_COMPENSATION_SCALAR );
        

        return ChassisSpeeds.fromRobotRelativeSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(xVelocity, yVelocity, angularVelocity),
                swerveInputs.Pose.getRotation()),

            swerveInputs.Pose.getRotation().plus(skewCompensationFactor));
 
    }



    public void setModuleStates(ChassisSpeeds speeds){
        this.chassisSpeeds = speeds;// I don't know anything else :(

        //change the chassis speed to module state to gwt the sepeed and angle of 4 wheels
        desiredModuleStates = kinematics.toSwerveModuleStates(speeds);



    }

    public void setWantedState(WantedState state){
        this.wantedState = state;
    }

    public SystemState handleStateTransition(){
        return switch(wantedState){
            case TELEOP_DRIVE -> SystemState.TELEOP_DRIVE;
            default -> SystemState.IDLE;
        };

    }


    public void applayState(){
        switch(systemState){
            default:
            case TELEOP_DRIVE:
                calculateSpeedsBasedOnJoystickInputs();
                setModuleStates();
        }
    }

    public SwerveModulePosition[] getSwerveModulePosition(){
        return swerveInputs.SwerveModulePosition;
    }

    

    
}
