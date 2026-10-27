package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;

/**
     * @param name A name for the device
     * @param deviceID The channel of the motor controller
     * @param motorType Specify whether the motor controller is Brushed or Brushless
     * @param idleMode Specify whether the motor controller is set to Coast or Brake mode
     * @param reverse Reverses the direction of the motor controller
     * @param positionFactor The ratio of encoder units to desired units (ie. units -> in)
*/

public class SparkMaxMotorController implements edu.wpi.first.wpilibj.motorcontrol.MotorController{
    private final SparkMax motor;
    public final SparkClosedLoopController pidController;
    private final RelativeEncoder encoder;
    private final String name;

    private final EncoderConfig encoderConfig;
    private final SparkMaxConfig sparkMaxConfig;

    public SparkMaxMotorController(String name, int deviceID, MotorType motorType, boolean inverted, IdleMode idleMode, double positionFactor){
        this.name = name;
        motor = new SparkMax(deviceID, motorType);
        pidController = motor.getClosedLoopController();

        //Encoder config
        encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(positionFactor);

        //SparkMax config
        sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.inverted(inverted);
        sparkMaxConfig.idleMode(idleMode);
        sparkMaxConfig.apply(encoderConfig);

        applyConfig();
        encoder = motor.getEncoder();
    }

    public SparkMaxMotorController(String name, int deviceID, MotorType motorType, boolean inverted, IdleMode idleMode){
        this(name, deviceID, motorType, inverted, idleMode, 1.0);
    }

    //Speed from -1.0 to 1.0 based on motor's available power
    @Override
    public void set(double speed){
        motor.set(speed);
    }

    @Override
    public double get(){
        return motor.get();
    }

    @Override
    public void stopMotor(){
        motor.set(0);
    }

    @Override
    public void setVoltage(double volts){
        double capped = MathUtil.clamp(volts, -12.0, 12.0);
        motor.setVoltage(capped);
    }

    @Override
    public boolean getInverted(){
        return motor.configAccessor.getInverted();
    }

    @Override
    public void setInverted(boolean inverted)
    {
        sparkMaxConfig.inverted(inverted);
        applyConfig();
    }

    //SparkMax specific
    public void resetEncoder(){
        encoder.setPosition(0);
    }

    public double getPosition(){
        return encoder.getPosition();
    }

    public void setPosition(double pos){
        encoder.setPosition(pos);
    }

    public double getVelocity(){
        return encoder.getVelocity();
    }

    public void setPID(double kP, double kI, double kD){
        sparkMaxConfig.closedLoop.p(kP).i(kI).d(kD).outputRange(-12, 12);
        applyConfig();
    }

    public void setFF(double kS, double kG, double kV)
    {
        sparkMaxConfig.closedLoop.feedForward.kS(kS).kG(kG).kCos(kG).kV(kV);
        applyConfig();
    }

    public void setPositionConversionFactor(double factor){
        encoderConfig.positionConversionFactor(factor);
        sparkMaxConfig.apply(encoderConfig);
        applyConfig();
    }

    public void setVelocityConversionFactor(double factor){
        encoderConfig.velocityConversionFactor(factor);
        sparkMaxConfig.apply(encoderConfig);
        applyConfig();
    }

    public void follow(SparkMaxMotorController leader){
        sparkMaxConfig.follow(leader.motor);
        applyConfig();
    }

    public void follow(SparkMaxMotorController leader, boolean invert){
        sparkMaxConfig.follow(leader.motor, invert);
        applyConfig();
    }

    private void applyConfig(){
        motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public String getName(){
        return name;
    }

    @Override
    public void disable() {
        stopMotor();
    }
}
