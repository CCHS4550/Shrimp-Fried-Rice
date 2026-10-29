package frc.robot.subsystems.Drive.module;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs
    {
        // tracking variables for the drive motor
        public boolean driveConnected = false;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        // tracking variables for the turn motor
        public boolean turnConnected = false;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
        public double turnEncoderVolts = 0.0;

        // tracking the odometry queues
        // public double[] odometryTimestamps = new double[] {};
        //public double[] odometryDrivePositionsRad = new double[] {};
        //public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    public default void setDriveVelocity(double velocity){}


    public default void setTurnPosition(double velocity){}

    
    default void updateModuleInputs(SparkMax driveMotor) {}
}
